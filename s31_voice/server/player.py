"""音乐链路：网易云 -> 本地缓存 -> RAOP 推给 Tivoli。

队列在**我们这边**，不在设备上。这是刻意的：设备那头我们什么都问不到
（没有厂商 API，Cast 的 :8009 四个 TLS 版本全被 RST），所以"现在放的是什么"
只能靠自己记账。好处是这笔账百分之百准 —— 是我们自己一首一首推过去的。

一条关键的顺序约束：**直链要在真要播那一刻才取**。网易云的直链是有时效的，
一次给整条队列取好，等放到第十首时前面九个都过期了。
"""
from __future__ import annotations

import asyncio
import logging
import os
import random
import tempfile
import time
from dataclasses import dataclass

import httpx

from airplay import AirPlay
from intent import Intent
from netease import Netease, NeteaseError, Song

_LOG = logging.getLogger("player")

_CACHE_DIR = os.environ.get(
    "MUSIC_CACHE_DIR", os.path.join(tempfile.gettempdir(), "s31_music"))
_CACHE_MAX_MB = int(os.environ.get("MUSIC_CACHE_MB", "512"))
# 推流时给设备设的音量。R2 说唤醒率在 SNR 5dB 掉到 1/6 —— 音乐一大声，板子就聋了。
# 这是"语音优先"的上限，用户嫌小可以再喊大声一点。
_DEFAULT_VOLUME = float(os.environ.get("MUSIC_VOLUME", "35"))


@dataclass
class NowPlaying:
    song: Song
    started_at: float
    offset_s: float = 0.0     # 从第几秒开始推的（续播用）

    @property
    def elapsed(self) -> float:
        return self.offset_s + (time.time() - self.started_at)


class MusicExecutor:
    """队列 + 播放循环。tiv 只用来在推流失败时做一次开机重试。"""

    def __init__(self, ne: Netease, air: AirPlay, tiv, llm=None) -> None:
        self.ne = ne
        self.air = air
        self.tiv = tiv
        self.llm = llm
        self.queue: list[Song] = []
        self.index = 0
        self.now: NowPlaying | None = None
        self._task: asyncio.Task | None = None
        self._paused: NowPlaying | None = None
        # 音量只在**开一条新队列**时设一次。早先每首歌都设，
        # 结果用户中途调大的音量，下一首开头就被打回默认值。
        self._volume_pending: float | None = None
        self._lock = asyncio.Lock()
        os.makedirs(_CACHE_DIR, exist_ok=True)

    # ---------------------------------------------------------- 缓存

    def _cache_path(self, song: Song, url: str) -> str:
        ext = ".mp3"
        for cand in (".flac", ".m4a", ".mp3"):
            if cand in url.split("?")[0]:
                ext = cand
                break
        return os.path.join(_CACHE_DIR, f"{song.id}{ext}")

    async def _fetch(self, song: Song, url: str) -> str:
        """整首下下来再推。

        不是边下边推：直链有时效，而且一旦推到一半断了，RAOP 那头会留下一个
        半开的会话，比多花两秒下载难收拾得多。局域网上一首 320k 的歌 ~10MB，
        下载是两三秒的事。
        """
        path = self._cache_path(song, url)
        if os.path.exists(path) and os.path.getsize(path) > 10240:
            return path
        tmp = path + ".part"
        async with httpx.AsyncClient(timeout=30.0, follow_redirects=True,
                                     trust_env=False) as c:
            async with c.stream("GET", url) as r:
                r.raise_for_status()
                with open(tmp, "wb") as f:
                    async for chunk in r.aiter_bytes(65536):
                        f.write(chunk)
        os.replace(tmp, path)
        self._trim_cache()
        return path

    def _trim_cache(self) -> None:
        try:
            files = [(os.path.join(_CACHE_DIR, f), os.path.getmtime(os.path.join(_CACHE_DIR, f)),
                      os.path.getsize(os.path.join(_CACHE_DIR, f)))
                     for f in os.listdir(_CACHE_DIR)]
        except OSError:
            return
        total = sum(s for _, _, s in files)
        for path, _, size in sorted(files, key=lambda x: x[1]):
            if total <= _CACHE_MAX_MB * 1024 * 1024:
                break
            try:
                os.remove(path)
                total -= size
            except OSError:
                pass

    # ---------------------------------------------------------- 播放循环

    async def _play_loop(self, start_offset: float = 0.0) -> None:
        offset = start_offset
        while 0 <= self.index < len(self.queue):
            song = self.queue[self.index]
            try:
                url = await self.ne.url(song.id)          # 播之前才取
            except NeteaseError as exc:
                _LOG.warning("取直链失败：%s", exc)
                url = None
            if not url:
                _LOG.info("跳过 %s：%s", song.label, self.ne.why_no_url(song))
                self.index += 1
                offset = 0.0
                continue

            try:
                path = await self._fetch(song, url)
            except Exception as exc:  # noqa: BLE001
                _LOG.warning("下载 %s 失败：%s，跳过", song.label, exc)
                self.index += 1
                offset = 0.0
                continue

            self.now = NowPlaying(song, time.time(), offset)
            _LOG.info("推流 %s（第 %d/%d 首）", song.label, self.index + 1, len(self.queue))
            try:
                await self._stream(path, song, offset)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                _LOG.warning("推流失败：%s", exc)
                # 推不过去最可能的原因是设备关着。让红外那条链把它弄醒再试一次 ——
                # 这正是锚定原语的价值：失败本身是有信息的。
                ok, _ = await self.tiv.ensure_wifi()
                if not ok:
                    self.now = None
                    return
                try:
                    await self._stream(path, song, offset)
                except Exception as exc2:  # noqa: BLE001
                    _LOG.warning("重试还是不行：%s，跳过这首", exc2)
            self.index += 1
            offset = 0.0
        self.now = None
        _LOG.info("队列放完了")

    async def _stream(self, path: str, song: Song, offset: float) -> None:
        from pyatv.interface import MediaMetadata

        meta = MediaMetadata(title=song.name, artist=song.artist, album=song.album,
                             duration=song.duration_ms / 1000.0 if song.duration_ms else None)
        src = path
        if offset > 1.0:
            src = _seek(path, offset, song.duration_ms / 1000.0)
        vol, self._volume_pending = self._volume_pending, None
        await self.air.stream(src, metadata=meta, volume=vol)

    def _start(self, offset: float = 0.0, set_volume: bool = False) -> None:
        if set_volume:
            self._volume_pending = _DEFAULT_VOLUME
        self._cancel()
        self._task = asyncio.create_task(self._play_loop(offset))

    def _cancel(self) -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
        self._task = None

    # ---------------------------------------------------------- 选歌

    async def _resolve(self, intent: Intent) -> tuple[list[Song], str]:
        """把意图变成一条队列。返回 (队列, 回话)。"""
        slots = intent.slots
        query = str(slots.get("query") or "").strip()

        if slots.get("vibe"):
            # 规则层抓不住的氛围类点歌。先让本地 LLM 出一个搜歌单的词，
            # 出不来（模型没装/超时）就拿原话去搜 —— 降级路径必须存在，
            # 不能因为模型没起来就整条哑掉。
            kw = None
            if self.llm is not None:
                kw = await self.llm.keywords(intent.raw or query)
            kw = kw or query
            pls = await self.ne.search_playlists(kw, limit=3)
            if pls:
                pid, name = pls[0]
                songs = await self.ne.playlist(pid, limit=60)
                random.shuffle(songs)
                if songs:
                    return songs, f"放一张叫《{name}》的歌单"
            songs = await self.ne.search(kw, limit=30)
            if songs:
                return songs, f"按「{kw}」找了几首"
            return [], f"没找到跟「{kw}」有关的歌"

        if slots.get("title"):
            # 歌名 + 歌手一起搜，比只搜歌名准得多（"红豆"有几十个版本）
            kw = " ".join(x for x in (slots.get("artist"), slots.get("title")) if x)
            songs = await self.ne.search(kw, limit=30)
            # 多一次往返换热度字段（搜索接口不回 pop，song/detail 才回）。
            # 没有它就只能按搜索排名挑，而搜索排名很差 —— 搜"红豆"时它把
            # 一堆翻唱排在王菲前面，按 pop 排立刻就对了。
            songs = await self.ne.songs([x.id for x in songs]) or songs
            picked = _best(songs, slots.get("artist"), slots.get("title"))
            if not picked:
                return [], f"没找到《{slots['title']}》"
            want = slots.get("artist")
            if want and not _is_artist(picked, want):
                # 找到了同名的歌，但不是他唱的。**必须说出来** —— 网易云上有大量
                # 翻唱，而某些歌手的原唱压根不在这个平台上（版权在别家）。
                # 默默放一个翻唱还说"播放周杰伦的稻香"，是在骗人。
                return [picked], f"网易云上没有{want}的《{slots['title']}》，给你放{picked.label}"
            return [picked], f"播放{picked.label}"

        if slots.get("artist") and not slots.get("title"):
            songs = await self.ne.search(slots["artist"], limit=30)
            # 只要**主唱**是他。搜"周杰伦"回来的前几条里有《布拉格广场》《刀马旦》——
            # 他确实挂在 artists 里，但那是蔡依林和李玟的歌，
            # 而"放周杰伦的歌"要的显然不是这个。
            songs = _by_artist(songs, slots["artist"])
            random.shuffle(songs)
            if songs:
                return songs, f"放{slots['artist']}的歌"
            return [], f"没找到{slots['artist']}"

        if query:
            songs = await self.ne.search(query, limit=20)
            if songs:
                return songs, f"播放{songs[0].label}"
            return [], f"没找到「{query}」"

        return [], "你想听什么？"

    # ---------------------------------------------------------- 对外

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        async with self._lock:
            try:
                return await self._execute(intent)
            except NeteaseError as exc:
                # 已经翻译成人话了，直接念
                return False, str(exc)
            except Exception as exc:  # noqa: BLE001
                _LOG.exception("音乐链路出错")
                return False, f"音乐这边出错了：{exc}"

    async def _execute(self, intent: Intent) -> tuple[bool, str]:
        a = intent.action

        if a == "play":
            songs, reply = await self._resolve(intent)
            if not songs:
                return False, reply
            self.queue, self.index, self._paused = songs, 0, None
            self._start(set_volume=True)
            return True, reply

        if a == "play_favorites":
            if not self.ne.logged_in:
                return False, "网易云还没登录，跑一下扫码登录那个脚本"
            ids = await self.ne.likelist()
            if not ids:
                return False, "你的收藏是空的"
            songs = await self.ne.songs(ids[:200])
            random.shuffle(songs)
            self.queue, self.index, self._paused = songs, 0, None
            self._start(set_volume=True)
            return True, f"放你收藏的歌，一共{len(songs)}首"

        if a in ("next", "prev"):
            if not self.queue:
                return False, "现在没在放歌"
            step = 1 if a == "next" else -1
            self.index = max(0, min(len(self.queue) - 1, self.index + step))
            self._paused = None
            self._start()
            song = self.queue[self.index]
            return True, f"{'下一首' if step > 0 else '上一首'}，{song.label}"

        if a == "pause":
            if self.now is None:
                return False, "现在没在放歌"
            # RAOP 是推流协议，没有"暂停"这回事 —— 暂停就是我们停止推。
            # 位置记在自己这儿，续播时从这个位置重新推。
            self._paused = NowPlaying(self.now.song, self.now.started_at, self.now.elapsed)
            self._cancel()
            self.now = None
            return True, "暂停了"

        if a == "resume":
            if self._paused is None:
                if self.queue and self.now is None:
                    self._start()
                    return True, "继续"
                return False, "没有暂停的歌"
            offset = self._paused.offset_s
            self._paused = None
            self._start(offset)
            return True, "继续"

        if a == "stop":
            self._cancel()
            self.now = None
            self._paused = None
            self.queue, self.index = [], 0
            return True, "停了"

        if a == "now_playing":
            if self.now is not None:
                s = self.now.song
                return True, f"{s.artist}的《{s.name}》"
            if self._paused is not None:
                return True, f"暂停在{self._paused.song.label}"
            return False, "现在没在放歌"

        return False, intent.reply or "这个我还不会"

    def status(self) -> dict:
        cur = self.now or self._paused
        return {
            "playing": self.now is not None,
            "paused": self._paused is not None,
            "queue_len": len(self.queue),
            "index": self.index,
            "song": None if cur is None else {
                "id": cur.song.id, "name": cur.song.name, "artist": cur.song.artist,
                "elapsed_s": round(cur.elapsed if cur is self.now else cur.offset_s, 1),
                "duration_s": round(cur.song.duration_ms / 1000.0, 1)},
        }


def _is_artist(song: Song, artist: str) -> bool:
    """这首歌算不算是他唱的。宽一点：合唱也算，但只是名字里带一样的字不算。"""
    return any(a == artist or (len(artist) > 1 and artist in a) for a in song.artists)


def _by_artist(songs: list[Song], artist: str) -> list[Song]:
    """按歌手筛，一层层放宽：主唱完全相等 -> 参演里有 -> 名字互相包含 -> 不筛了。

    一层层放宽而不是一把抓，是因为这几档的含义差别很大：主唱是他 = 他的歌；
    只是参演 = 别人的歌他来了一段。降级到最后一档时结果已经不太可信了，
    但那时候的替代品是"什么都没找到"，还不如给几首。
    """
    primary = [s for s in songs if s.artists and s.artists[0] == artist]
    if primary:
        return primary
    featured = [s for s in songs if any(a == artist for a in s.artists)]
    if featured:
        return featured
    loose = [s for s in songs if any(artist in a or a in artist for a in s.artists)]
    return loose or songs


def _best(songs: list[Song], artist: str | None, title: str | None) -> Song | None:
    """搜索结果里挑最像用户要的那首。

    筛的顺序是 **歌名先于歌手**，这一点是踩出来的：先按歌手筛的话，
    「播放周杰伦的稻香」会筛出一堆周杰伦的歌、再在里面找不到《稻香》、
    然后退回去按热度挑 —— 结果放了《刀马旦》。歌名是用户说得最具体的那个词，
    它筛不出东西就该认输，而不是拿同一个歌手的另一首歌顶上。
    """
    if not songs:
        return None
    cands = songs
    if title:
        # 「红豆」和「红豆生南国」是两首歌，完全相等优先；
        # 「稻香 (女声版)」这类带后缀的退而求其次。都没有就是真没有。
        cands = ([s for s in cands if s.name == title]
                 or [s for s in cands if title in s.name])
        if not cands:
            return None
    if artist:
        cands = _by_artist(cands, artist)
    # 热度优先，同热度选长的（短的通常是片段/伴奏/彩铃）。
    # 热度这一步不能省：搜「红豆」时搜索排名把一堆翻唱排在王菲前面。
    return max(cands, key=lambda s: (s.pop, s.duration_ms))


def _seek(path: str, offset_s: float, duration_s: float) -> str:
    """从第 offset_s 秒续播：切出一个临时文件，返回它的路径。

    返回**路径**而不是文件句柄，是因为播放那头是 afplay（见 airplay.py 换实现的原因），
    它只认路径。多写一个临时文件，换的是不用自己灌 CoreAudio。

    MP3 没有索引，按字节比例定位是**近似**的（VBR 会偏），所以再往后找一个
    帧同步头对齐，避免解码器从半个帧中间开始。误差在一两秒内，续播这个场景够用。
    非 MP3（网易云偶尔给 m4a/flac）没这个技巧可用，就从头放 ——
    宁可重来一遍，也不要送一段解不开的字节流过去。
    """
    if not path.lower().endswith(".mp3") or duration_s <= 0:
        return path
    size = os.path.getsize(path)
    approx = int(size * min(0.98, offset_s / duration_s))
    with open(path, "rb") as f:
        f.seek(approx)
        window = f.read(8192)
        pos = 0
        while pos + 1 < len(window):
            if window[pos] == 0xFF and (window[pos + 1] & 0xE0) == 0xE0:
                break
            pos += 1
        else:
            pos = 0
        f.seek(approx + pos)
        tail = f.read()
    out = os.path.join(_CACHE_DIR, f"_resume_{os.path.basename(path)}")
    with open(out, "wb") as f:
        f.write(tail)
    return out
