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


# 播放模式。和网易云的四档一致，用户的心理模型是现成的，不用重新教。
SEQUENTIAL = "sequential"   # 顺序播放：放完就停
REPEAT_ALL = "repeat_all"   # 列表循环：放完从头再来
SHUFFLE    = "shuffle"      # 随机播放：打乱顺序，放完重新打乱再来
REPEAT_ONE = "repeat_one"   # 单曲循环：一直放这一首

_MODE_CN = {SEQUENTIAL: "顺序播放", REPEAT_ALL: "列表循环",
            SHUFFLE: "随机播放", REPEAT_ONE: "单曲循环"}


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
        # 播放顺序**不是**靠打乱 queue 本身来实现的。早先随机播放是直接
        # random.shuffle(queue)，副作用是原始顺序当场丢失 —— 用户之后想切回
        # 「顺序播放」就再也回不去了。现在 queue 永远保持原序，
        # 打乱的是这张索引表，切模式只要重建它。
        self._order: list[int] = []
        self._pos = 0
        self.mode = SHUFFLE
        self.now: NowPlaying | None = None
        self._task: asyncio.Task | None = None
        self._paused: NowPlaying | None = None
        # 音量只在**开一条新队列**时设一次。早先每首歌都设，
        # 结果用户中途调大的音量，下一首开头就被打回默认值。
        self._volume_pending: float | None = None
        self._link_lost = False
        # 上一次"什么都没放成"的原因。单曲请求时要拿它当回话 ——
        # 用户说了一首具体的歌，结果没放，必须告诉他为什么。
        self.last_skip_reason = ""
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

    # ---------------------------------------------------------- 播放顺序

    def _rebuild_order(self, keep_current: bool = True) -> None:
        """按当前模式重建播放顺序表。keep_current 时把正在放的那首挪到最前面，
        这样切模式不会打断当前这首歌 —— 切模式是"接下来怎么放"，不是"重放"。"""
        n = len(self.queue)
        self._order = list(range(n))
        if self.mode == SHUFFLE:
            random.shuffle(self._order)
        if keep_current and 0 <= self.index < n and self._order:
            self._order.remove(self.index)
            self._order.insert(0, self.index)
        self._pos = 0

    def _advance(self) -> bool:
        """挪到下一首。返回 False 表示放完了该停。"""
        if self.mode == REPEAT_ONE:
            return True                      # 原地不动，重放这一首
        self._pos += 1
        if self._pos < len(self._order):
            self.index = self._order[self._pos]
            return True
        if self.mode == SEQUENTIAL:
            return False                     # 顺序播放到底就停
        # 列表循环 / 随机：从头再来。随机时重新洗一次牌，
        # 否则第二轮的顺序和第一轮一模一样，听起来根本不像随机。
        self._rebuild_order(keep_current=False)
        if not self._order:
            return False
        self.index = self._order[0]
        return True

    # ---------------------------------------------------------- 链路

    async def _ensure_link(self) -> tuple[bool, str]:
        """放歌之前必须确认声音**真的会送到 Tivoli**。

        这一步是踩出来的：afplay 是往"当前系统默认输出"放的，而它不管
        设备在不在、链路通不通都返回 0。所以链路断了的时候，整条命令看起来
        完全成功 —— 服务端日志写着"推流 王菲的《红豆》"、接口回 ok=true ——
        而歌其实从 MacBook 自己的喇叭里出来了。

        链路什么时候会断：用红外把源从 WiFi 切到 FM（听广播）、Tivoli 断电、
        或者有人手动切走 Mac 的输出。也就是说**正常用一次收音机就会断**，
        这不是异常路径，是主路径。
        """
        if await self.air.linked():
            return True, ""
        if not await self.air.reachable():
            return False, "音响不在线，先打开它"
        _LOG.info("AirPlay 链路断了（多半是刚才切过源），重新挂上去……")
        if await self.air.relink():
            return True, ""
        return False, "接不上音响的 AirPlay，声音会从电脑里出来，所以我没放"

    # ---------------------------------------------------------- 播放循环

    async def _play_loop(self, start_offset: float = 0.0) -> None:
        offset = start_offset
        retries_left = 2
        skipped: list[str] = []
        guard = 0
        while 0 <= self.index < len(self.queue):
            # 循环模式下这个 while 是真的无限循环，所以要防住"整张列表全跳过"
            # 的情况（全是 VIP 试听、或者全都下不动）—— 否则它会空转烧 CPU。
            guard += 1
            if guard > max(len(self.queue) * 2, 20):
                _LOG.warning("连着跳过太多首，停下来")
                self.now = None
                return
            song = self.queue[self.index]
            try:
                info = (await self.ne.play_info([song.id])).get(song.id)   # 播之前才取
            except NeteaseError as exc:
                _LOG.warning("取直链失败：%s", exc)
                info = None
            if info is None:
                skipped.append(self.ne.why_no_url(song))
                _LOG.info("跳过 %s：%s", song.label, skipped[-1])
                offset = 0.0
                if not self._advance():
                    break
                continue
            if info.is_trial:
                # VIP 曲目在无会员时**照样给 url**，只是给的是试听片段。
                # 放一段然后戛然而止，比不放更让人困惑 —— 所以跳过并记下原因。
                skipped.append(self.ne.why_trial(song, info))
                _LOG.info("跳过 %s：%s", song.label, skipped[-1])
                offset = 0.0
                if not self._advance():
                    break
                continue
            url = info.url

            try:
                path = await self._fetch(song, url)
            except Exception as exc:  # noqa: BLE001
                _LOG.warning("下载 %s 失败：%s，跳过", song.label, exc)
                offset = 0.0
                if not self._advance():
                    break
                continue

            # 每首歌之前都再确认一次。一首歌四分钟，这期间用户完全可能
            # 去听了会儿收音机又回来 —— 那时链路已经断了。
            ok, why = await self._ensure_link()
            if not ok:
                _LOG.warning("链路不通，停止播放：%s", why)
                self.now = None
                return
            self.now = NowPlaying(song, time.time(), offset)
            _LOG.info("推流 %s（第 %d/%d 首）", song.label, self.index + 1, len(self.queue))
            try:
                await self._stream_guarded(path, song, offset)
                if self._link_lost:
                    # 播放中途链路断了（2026-09-12 遇到：一首 4 分 20 的歌放到
                    # 50 秒时 AirPlay 设备从 CoreAudio 里消失，afplay 被打断，
                    # 声音掉回 MacBook 喇叭）。根因还没查清 —— 查它必须出声。
                    # 但症状可以先堵：停下来、挂回去、从断点续，最多试两次。
                    self._link_lost = False
                    resume_at = self.now.elapsed if self.now else 0.0
                    if retries_left <= 0:
                        _LOG.warning("链路反复断开，放弃这首")
                        self.now = None
                        return
                    retries_left -= 1
                    ok, why = await self._ensure_link()
                    if not ok:
                        _LOG.warning("续不上：%s", why)
                        self.now = None
                        return
                    _LOG.info("链路挂回来了，从 %.0f 秒处续播", resume_at)
                    offset = resume_at
                    continue        # 不推进 index，重播这首
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
            guard = 0          # 成功放完一首，跳过计数清零
            offset = 0.0
            if not self._advance():
                break
        self.now = None
        if skipped and self.index >= len(self.queue):
            # 整条队列一首都没放成，原因要说出来，不能只是安静地结束
            self.last_skip_reason = skipped[0]
            _LOG.info("队列里 %d 首都没放成，第一条原因：%s", len(skipped), skipped[0])
        else:
            _LOG.info("队列放完了")

    async def _stream_guarded(self, path: str, song: Song, offset: float) -> None:
        """推一首歌，同时盯着链路。

        为什么要盯：`afplay` 往的是"系统当前默认输出"，而 AirPlay 一断，
        macOS 会**静默地**把输出迁回内置喇叭 —— afplay 不报错、不退出，
        歌就这么从电脑里放出来了。整条链上没有任何一层会告诉我们这件事，
        所以只能自己每隔几秒回头看一眼。
        """
        self._link_lost = False
        watchdog = asyncio.create_task(self._watch_link())
        try:
            await self._stream(path, song, offset)
        finally:
            watchdog.cancel()

    async def _watch_link(self) -> None:
        while True:
            await asyncio.sleep(4)
            if not await self.air.linked():
                _LOG.warning("播放中途 AirPlay 链路断了，立刻停 —— "
                             "不能让声音接着从电脑喇叭里出来")
                self._link_lost = True
                await self.air.stop()
                return

    async def _stream(self, path: str, song: Song, offset: float) -> None:
        from pyatv.interface import MediaMetadata

        meta = MediaMetadata(title=song.name, artist=song.artist, album=song.album,
                             duration=song.duration_ms / 1000.0 if song.duration_ms else None)
        src = path
        if offset > 1.0:
            src = _seek(path, offset, song.duration_ms / 1000.0)
        vol, self._volume_pending = self._volume_pending, None
        await self.air.stream(src, metadata=meta, volume=vol)

    def _start(self, offset: float = 0.0, set_volume: bool = False,
               new_queue: bool = False) -> None:
        if set_volume:
            self._volume_pending = _DEFAULT_VOLUME
        if new_queue:
            self.index = 0
            self._rebuild_order(keep_current=False)
            if self._order:
                self.index = self._order[0]
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
            # 先确认链路再去查歌：查歌要一两秒，而链路不通的话这一两秒白花，
            # 更糟的是用户会先听到"播放王菲的《红豆》"然后声音从电脑里出来。
            ok, why = await self._ensure_link()
            if not ok:
                return False, why
            songs, reply = await self._resolve(intent)
            if not songs:
                return False, reply
            # 单曲请求要在**回话之前**就确认它能不能完整播放。
            # 否则用户先听到"播放王菲的《红豆》"，然后 45 秒戛然而止。
            if len(songs) == 1:
                try:
                    info = (await self.ne.play_info([songs[0].id])).get(songs[0].id)
                except NeteaseError as exc:
                    return False, str(exc)
                if info is None:
                    return False, self.ne.why_no_url(songs[0])
                if info.is_trial:
                    return False, self.ne.why_trial(songs[0], info)
            self.queue, self.index, self._paused = songs, 0, None
            self._start(set_volume=True, new_queue=True)
            return True, reply

        if a == "play_favorites":
            ok, why = await self._ensure_link()
            if not ok:
                return False, why
            if not self.ne.logged_in:
                return False, "网易云还没登录，跑一下扫码登录那个脚本"
            ids = await self.ne.likelist()
            if not ids:
                return False, "你的收藏是空的"
            songs = await self.ne.songs(ids[:200])
            self.queue, self.index, self._paused = songs, 0, None
            self._start(set_volume=True, new_queue=True)
            return True, f"放你收藏的歌，一共{len(songs)}首"

        if a in ("next", "prev"):
            if not self.queue:
                return False, "现在没在放歌"
            ok, why = await self._ensure_link()
            if not ok:
                return False, why
            # 手动切歌走的是**播放顺序表**，不是 index±1 ——
            # 随机模式下 index±1 会切到列表里相邻的那首，而不是随机序列的下一首，
            # 用户会觉得"随机播放的下一首怎么是按顺序的"。
            if a == "next":
                if not self._advance():
                    return False, "已经是最后一首了"
            else:
                if self._pos > 0:
                    self._pos -= 1
                    self.index = self._order[self._pos]
                elif self.mode in (REPEAT_ALL, SHUFFLE):
                    self._pos = len(self._order) - 1
                    self.index = self._order[self._pos]
                else:
                    return False, "已经是第一首了"
            self._paused = None
            self._start()
            song = self.queue[self.index]
            return True, f"{'下一首' if a == 'next' else '上一首'}，{song.label}"

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
            ok, why = await self._ensure_link()
            if not ok:
                return False, why
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

        if a == "set_mode":
            mode = intent.slots.get("mode")
            if mode not in _MODE_CN:
                return False, "这个播放模式我不认识"
            self.mode = mode
            if self.queue:
                # 只重建顺序表，**不打断正在放的这一首** ——
                # 「换成随机播放」是在说接下来怎么放，不是"重放一遍"。
                self._rebuild_order(keep_current=True)
            return True, f"换成{_MODE_CN[mode]}"

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
            "mode": self.mode,
            "mode_cn": _MODE_CN.get(self.mode, self.mode),
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
