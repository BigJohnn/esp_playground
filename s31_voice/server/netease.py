"""网易云音乐。**整条腐化风险都关在这一个文件里**（roadmap 的 R5）。

这是全套方案里唯一一条会自己坏掉的链路：cookie 会过期、接口会变、
歌会下架、VIP 会拦。所以隔离原则是硬的 —— 这里坏掉不许牵连 FM 和灯，
而且失败必须说人话（"这首要会员"，不是"播放失败"）。

不用 NeteaseCloudMusicApi（那个 Node 项目已经从 npm 下架了，剩下的是各种分叉），
直接说 weapi 协议：AES 两次 + RSA 包一次密钥。加密层就三十行，
比多养一个 Node 进程、多一个会自己坏的依赖要省心得多，pycryptodome 本来就在 venv 里。

cookie 存在 .netease_cookie（已进 .gitignore），和 MIIO_TOKEN 一个待遇。
拿 cookie 的办法是扫码：tools/netease_login.py
"""
from __future__ import annotations

import base64
import json
import logging
import os
import secrets
from dataclasses import dataclass, field

import httpx

_LOG = logging.getLogger("netease")

_BASE = "https://music.163.com"
_UA = ("Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 "
       "(KHTML, like Gecko) Chrome/120.0 Safari/537.36")

# weapi 的两个固定量，公开常量，不是密钥
_PRESET_KEY = b"0CoJUm6Qyw8W8jud"
_IV = b"0102030405060708"
_PUBKEY_N = int(
    "00e0b509f6259df8642dbc35662901477df22677ec152b5ff68ace615bb7b725152b3ab17a876aea8a5"
    "aa76d2e417629ec4ee341f56135fccf695280104e0312ecbda92557c93870114af6c9d05c4f7f0c3685"
    "b7a46bee255932575cce10b424d813cfe4875d3e82047b97ddef52741d546b8e289dc6935b3ece0462d"
    "b0a22b8e7", 16)
_PUBKEY_E = 0x10001

_COOKIE_PATH = os.environ.get(
    "NETEASE_COOKIE_PATH", os.path.join(os.path.dirname(__file__), ".netease_cookie"))


class NeteaseError(RuntimeError):
    """已经翻译成人话的失败。message 可以直接念给用户听。"""


# ---------------------------------------------------------------- 加密

def _aes(text: bytes, key: bytes) -> bytes:
    from Crypto.Cipher import AES

    pad = 16 - len(text) % 16
    text = text + bytes([pad]) * pad
    return base64.b64encode(AES.new(key, AES.MODE_CBC, _IV).encrypt(text))


def _rsa(text: str) -> str:
    # 网易这里用的是**无填充**的裸 RSA，而且明文要先反转 —— 不是标准 PKCS#1，
    # 所以只能手算，不能用库的 encrypt()
    n = int(text[::-1].encode().hex(), 16)
    return format(pow(n, _PUBKEY_E, _PUBKEY_N), "x").zfill(256)


def weapi(payload: dict) -> dict:
    """把请求体包成 weapi 的 {params, encSecKey}。"""
    text = json.dumps(payload, ensure_ascii=False).encode()
    secret = secrets.token_hex(8).encode()      # 16 个十六进制字符
    return {"params": _aes(_aes(text, _PRESET_KEY), secret).decode(),
            "encSecKey": _rsa(secret.decode())}


# ---------------------------------------------------------------- 数据

@dataclass
class Song:
    id: int
    name: str
    artists: list[str] = field(default_factory=list)
    album: str = ""
    duration_ms: int = 0
    fee: int = 0            # 1=VIP  4=付费专辑  8=免费/低音质  0=免费
    # 热度 0-100。**只有 song/detail 会回**，搜索接口不回。
    # 挑版本全靠它：搜"稻香"回来前六条全是翻唱，热度一排原唱立刻浮上来。
    pop: float = 0.0
    url: str | None = None

    @property
    def artist(self) -> str:
        return "、".join(self.artists) if self.artists else "未知歌手"

    @property
    def label(self) -> str:
        return f"{self.artist}的《{self.name}》"

    @classmethod
    def from_json(cls, d: dict) -> "Song":
        ar = d.get("ar") or d.get("artists") or []
        al = d.get("al") or d.get("album") or {}
        return cls(
            id=int(d["id"]),
            name=str(d.get("name", "")),
            artists=[str(a.get("name", "")) for a in ar if a.get("name")],
            album=str(al.get("name", "")) if isinstance(al, dict) else "",
            duration_ms=int(d.get("dt") or d.get("duration") or 0),
            fee=int(d.get("fee") or 0),
            pop=float(d.get("pop") or 0.0),
        )


# ---------------------------------------------------------------- 客户端

class Netease:
    def __init__(self, cookie_path: str | None = None) -> None:
        self.cookie_path = cookie_path or _COOKIE_PATH
        self._cookie = self._load_cookie()
        self._uid: int | None = None
        self._client: httpx.AsyncClient | None = None

    # ---------- cookie ----------

    def _load_cookie(self) -> str:
        env = os.environ.get("NETEASE_COOKIE", "").strip()
        if env:
            return env
        try:
            with open(self.cookie_path, encoding="utf-8") as f:
                return f.read().strip()
        except FileNotFoundError:
            return ""

    def save_cookie(self, cookie: str) -> None:
        with open(self.cookie_path, "w", encoding="utf-8") as f:
            f.write(cookie.strip() + "\n")
        os.chmod(self.cookie_path, 0o600)
        self._cookie = cookie.strip()
        self._uid = None

    @property
    def logged_in(self) -> bool:
        return "MUSIC_U=" in self._cookie

    # ---------- 传输 ----------

    def _new_client(self) -> httpx.AsyncClient:
        cookie = self._cookie
        # os=pc 不是可选的：不带它，取播放直链的接口会一律回 null，
        # 而且不报错 —— 表现成"每首歌都没版权"，极难往这个方向想。
        for extra in ("os=pc", "appver=8.9.70"):
            if extra.split("=")[0] + "=" not in cookie:
                cookie = (cookie + "; " + extra).strip("; ")
        return httpx.AsyncClient(
            base_url=_BASE, timeout=12.0, follow_redirects=True,
            headers={"User-Agent": _UA, "Referer": _BASE + "/",
                     "Origin": _BASE, "Cookie": cookie,
                     "Content-Type": "application/x-www-form-urlencoded"},
            # 本机常年挂着代理（README §4.1.12 那次事故的同一个源头）。
            # 走代理会平白多一跳，而且代理挂了整条音乐链跟着挂 —— 这里直连。
            trust_env=False)

    async def _post(self, path: str, payload: dict) -> dict:
        if self._client is None:
            self._client = self._new_client()
        body = dict(payload)
        body.setdefault("csrf_token", self._csrf())
        try:
            r = await self._client.post(path, data=weapi(body))
            r.raise_for_status()
            data = r.json()
        except httpx.HTTPError as exc:
            raise NeteaseError("连不上网易云") from exc
        except ValueError as exc:
            raise NeteaseError("网易云返回的不是 JSON，接口大概是变了") from exc
        code = data.get("code")
        if code == 301 or code == -462:
            raise NeteaseError("网易云的登录过期了，重新扫一次码")
        return data

    def _csrf(self) -> str:
        for part in self._cookie.split(";"):
            part = part.strip()
            if part.startswith("__csrf="):
                return part[7:]
        return ""

    async def close(self) -> None:
        if self._client is not None:
            await self._client.aclose()
            self._client = None

    def reset(self) -> None:
        """cookie 换了之后把连接池丢掉 —— Cookie 头是建 client 时定死的。"""
        self._cookie = self._load_cookie()
        self._uid = None
        self._client = None

    # ---------- 接口 ----------

    async def account(self) -> dict | None:
        """当前登录的是谁。顺带把 uid 缓存下来，"我的收藏"要用。"""
        data = await self._post("/weapi/w/nuser/account/get", {})
        profile = data.get("profile")
        if not profile:
            return None
        self._uid = int(profile["userId"])
        return {"uid": self._uid, "nickname": profile.get("nickname", "")}

    async def uid(self) -> int:
        if self._uid is None:
            acc = await self.account()
            if acc is None:
                raise NeteaseError("网易云还没登录，先扫码")
        return int(self._uid)

    # 用老的 /weapi/search/get，不是 cloudsearch。cloudsearch 现在一律回
    # {"code":50000005}（风控），而 search/get 照常工作 —— 只是字段名是老一套
    # （artists/album/duration 而不是 ar/al/dt），Song.from_json 两套都认。
    async def search(self, keyword: str, limit: int = 12) -> list[Song]:
        data = await self._post("/weapi/search/get",
                                {"s": keyword, "type": 1, "offset": 0, "limit": limit,
                                 "total": True})
        songs = ((data.get("result") or {}).get("songs")) or []
        return [Song.from_json(s) for s in songs]

    async def search_playlists(self, keyword: str, limit: int = 6) -> list[tuple[int, str]]:
        """按关键词搜歌单。'放点适合睡觉的歌'最后落到这儿 —— 歌单比单曲搜索
        更接近"一段时间的听感"，而那正是氛围类请求真正想要的东西。"""
        data = await self._post("/weapi/search/get",
                                {"s": keyword, "type": 1000, "offset": 0, "limit": limit})
        pls = ((data.get("result") or {}).get("playlists")) or []
        return [(int(p["id"]), str(p.get("name", ""))) for p in pls]

    async def playlist(self, playlist_id: int, limit: int = 200) -> list[Song]:
        data = await self._post("/weapi/v6/playlist/detail",
                                {"id": int(playlist_id), "n": limit, "s": 0})
        pl = data.get("playlist") or {}
        tracks = pl.get("tracks") or []
        if tracks:
            return [Song.from_json(t) for t in tracks[:limit]]
        # 长歌单只回 trackIds，要再拉一次详情
        ids = [int(t["id"]) for t in (pl.get("trackIds") or [])][:limit]
        return await self.songs(ids) if ids else []

    async def songs(self, ids: list[int]) -> list[Song]:
        if not ids:
            return []
        out: list[Song] = []
        for i in range(0, len(ids), 500):
            chunk = ids[i:i + 500]
            data = await self._post(
                "/weapi/v3/song/detail",
                {"c": json.dumps([{"id": int(x)} for x in chunk]),
                 "ids": json.dumps([int(x) for x in chunk])})
            out += [Song.from_json(s) for s in (data.get("songs") or [])]
        # 详情接口不保证顺序，而歌单的顺序是用户自己排的，得还回去
        order = {sid: n for n, sid in enumerate(ids)}
        out.sort(key=lambda s: order.get(s.id, 1 << 30))
        return out

    async def likelist(self) -> list[int]:
        """红心歌曲的 id 列表。"播放我的收藏"就是这个。"""
        data = await self._post("/weapi/song/like/get", {"uid": await self.uid()})
        return [int(x) for x in (data.get("ids") or [])]

    async def url(self, song_id: int, br: int = 320000) -> str | None:
        """取播放直链。**必须在真要播之前才取** —— 这个链接是有时效的，
        提前给整条队列取好，等放到第十首时前面九个都过期了。"""
        urls = await self.urls([song_id], br)
        return urls.get(int(song_id))

    async def urls(self, ids: list[int], br: int = 320000) -> dict[int, str]:
        if not ids:
            return {}
        data = await self._post("/weapi/song/enhance/player/url",
                                {"ids": json.dumps([int(x) for x in ids]), "br": br})
        out: dict[int, str] = {}
        for d in (data.get("data") or []):
            if d.get("url"):
                out[int(d["id"])] = str(d["url"])
        return out

    @staticmethod
    def why_no_url(song: Song) -> str:
        """取不到直链时说人话。这条比什么都重要 —— R5 里唯一能做的缓解就是
        把失败讲清楚，让用户知道该换一首还是该去续会员。"""
        if song.fee in (1, 4):
            return f"《{song.name}》要会员才能听"
        return f"《{song.name}》拿不到播放链接，多半是没版权"


# ---------------------------------------------------------------- 扫码登录

async def qr_login_start(client: httpx.AsyncClient) -> tuple[str, str]:
    """返回 (unikey, 要编成二维码的 URL)。"""
    r = await client.post("/weapi/login/qrcode/unikey", data=weapi({"type": 1}))
    key = r.json()["unikey"]
    return key, f"{_BASE}/login?codekey={key}"


async def qr_login_poll(client: httpx.AsyncClient, unikey: str) -> tuple[int, str]:
    """返回 (code, cookie)。800=过期 801=等扫 802=已扫等确认 803=成功。"""
    r = await client.post("/weapi/login/qrcode/client/login",
                          data=weapi({"key": unikey, "type": 1}))
    code = int(r.json().get("code", 0))
    if code != 803:
        return code, ""
    jar = "; ".join(f"{c.name}={c.value}" for c in client.cookies.jar
                    if c.name in ("MUSIC_U", "__csrf", "NMTID", "__remember_me"))
    return code, jar


def login_client() -> httpx.AsyncClient:
    return httpx.AsyncClient(base_url=_BASE, timeout=12.0, trust_env=False,
                             headers={"User-Agent": _UA, "Referer": _BASE + "/",
                                      "Cookie": "os=pc; appver=8.9.70"})
