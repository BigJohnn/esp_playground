"""S31 Voice 服务端。

对外接口：
  GET  /health           活着没（唯一免鉴权的接口）
  GET  /commands         命令词表（板子开机拉一次，之后按版本增量刷新）
  GET  /commands/version 只回版本号，几十字节 —— 板子轮询用这个，不是上面那个
  POST /stt              裸 PCM / wav -> 文本
  POST /tts              文本 -> 16k PCM
  POST /wake             板子听到唤醒词就打这条，服务端立刻把音乐压低（见下面的注释）
  POST /command          文本 -> 解析意图、控设备、返回回话（板上 MultiNet 认出来后走这条）
  POST /utterance        整段 PCM -> STT -> 控设备（板上没认出来的兜底）
  GET  /devices          三台设备各自的状态（Tivoli 那栏是影子状态，不是真相）
  POST /tivoli/anchor    推一段静音锚定，看设备在不在 WiFi 源上（M6 排查用）
  POST /tivoli/ir/{键}   直接发一条红外（M6 实测用）
  POST /tivoli/hold_frames  改板上"长按连发多少帧"（M6 扫这个值用）
  GET  /music/status     当前队列和正在放的歌
  WS   /voice            旧的双向通道，板子已经不用了

先能用命令行/HTTP 单测每一段，再接板子。

除 /health 外全部要 `Authorization: Bearer $API_TOKEN`（API_TOKEN 留空则不鉴权）。
所有会控灯的接口都回一个 `ms` 字段，把耗时按阶段拆开 —— 只有一个端到端数字的时候，
"慢"这件事无法定位到底是上传、STT、还是灯泡自己慢。
"""
from __future__ import annotations

import asyncio
import hashlib
import hmac
import io
import json
import logging
import os
import time
import wave
from contextlib import asynccontextmanager

import numpy as np
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response

import discovery
import intent as intent_mod
from aircon import AirconExecutor
from airplay import AirPlay
from config import CONFIG
from executor import LightExecutor, Router
from ha import HomeAssistant
from llm import LocalLLM
from netease import Netease
from player import MusicExecutor
from stt import STT
from tivoli import TivoliExecutor
from tts import TTS

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)-8s %(message)s",
    datefmt="%H:%M:%S",
)
_LOG = logging.getLogger("app")

_ha: HomeAssistant
_exec: LightExecutor
_router: Router
_air: AirPlay
_tivoli: TivoliExecutor
_music: MusicExecutor
_ne: Netease
_llm: LocalLLM
_aircon: AirconExecutor


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _ha, _exec, _router, _air, _tivoli, _music, _ne, _llm, _aircon
    _ha = HomeAssistant()
    _exec = LightExecutor(_ha)
    if not CONFIG.ha_token:
        _LOG.warning("HA_TOKEN 没设，控灯会失败。HA -> 头像 -> 安全 -> 长期访问令牌")

    _air = AirPlay(host=CONFIG.airplay_host, name=CONFIG.airplay_name,
                   password=CONFIG.airplay_password)
    _tivoli = TivoliExecutor(_ha, _air)
    _ne = Netease()
    _llm = LocalLLM()
    _music = MusicExecutor(_ne, _air, _tivoli, _llm)
    _aircon = AirconExecutor(_ha)
    _router = Router(_exec, tivoli=_tivoli, music=_music, aircon=_aircon, llm=_llm)

    if not CONFIG.airplay_password:
        _LOG.warning("AIRPLAY_PASSWORD 没设，推流会被拒 —— 密码在设备自己的配置页上")
    if not _ne.logged_in:
        _LOG.warning("网易云没登录，「播放我的收藏」用不了。跑 tools/netease_login.py")
    if not _tivoli.conf.get("measured"):
        _LOG.warning("tivoli.json 的常数还没实测（measured=false），FM 那条链会拒绝动作")

    _disc_thread, _disc_stop = discovery.start(CONFIG.port)
    # miIO 握手保活。不这么做的话，每条隔了一分钟以上的命令都要先重握，
    # 那一下几百毫秒全落在用户等待里 —— 而控灯本来就是整条链上最慢的一段。
    warm_task = asyncio.create_task(_exec.keep_warm())
    # 模型预热放后台。它可能要几十秒（第一次加载 3G 权重），
    # 而这期间服务端必须已经能控灯 —— 兜底层不该挡住主路径起步。
    llm_task = asyncio.create_task(_llm.warm())
    yield
    warm_task.cancel()
    llm_task.cancel()
    _disc_stop.set()
    await _air.close()
    await _ne.close()
    await _llm.close()
    await _ha.close()


app = FastAPI(title="S31 Voice", lifespan=lifespan)

# /health 故意不鉴权：它是"服务还在不在"的探针，被人探到不算泄密。
# 详细信息（控哪只灯）不放在这儿，那需要令牌。
_OPEN_PATHS = {"/health"}


@app.middleware("http")
async def require_token(request: Request, call_next):
    """同一局域网里谁都能 POST 一条命令控灯 —— 这是之前的实际状态。

    共享令牌足够：威胁模型是"同网段的其他设备/人"，不是拿到了服务器本机权限的人。
    留空 API_TOKEN 就完全关掉鉴权，保持老用法能跑。
    """
    if CONFIG.api_token and request.url.path not in _OPEN_PATHS:
        # compare_digest 而不是 ==：令牌比较要常数时间，否则逐字节的提前返回
        # 会把正确前缀的长度泄露出去。
        if not hmac.compare_digest(request.headers.get("authorization", ""),
                                   f"Bearer {CONFIG.api_token}"):
            _LOG.warning("拒绝未授权请求 %s %s（来自 %s）", request.method,
                         request.url.path, request.client.host if request.client else "?")
            return JSONResponse({"error": "unauthorized"}, status_code=401)
    return await call_next(request)


def _fmt_ms(marks: dict[str, int]) -> str:
    return " ".join(f"{k}={v}ms" for k, v in marks.items())


class _Timer:
    """按阶段记毫秒。用 perf_counter 而不是 time()：后者会被 NTP 校时拽着走。"""

    def __init__(self) -> None:
        self._t0 = self._last = time.perf_counter()
        self.marks: dict[str, int] = {}

    def mark(self, name: str) -> None:
        now = time.perf_counter()
        self.marks[name] = round((now - self._last) * 1000)
        self._last = now

    def done(self) -> dict[str, int]:
        self.marks["server"] = round((time.perf_counter() - self._t0) * 1000)
        return self.marks


def _pcm_to_wav(pcm: bytes, rate: int) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


def _read_audio(body: bytes) -> tuple[np.ndarray, int]:
    """接受 wav 或 16k 裸 PCM。"""
    if body[:4] == b"RIFF":
        with wave.open(io.BytesIO(body), "rb") as w:
            rate = w.getframerate()
            frames = w.readframes(w.getnframes())
            data = np.frombuffer(frames, dtype="<i2")
            if w.getnchannels() > 1:
                data = data.reshape(-1, w.getnchannels()).mean(axis=1)
        return data.astype(np.float32) / 32768.0, rate
    return np.frombuffer(body, dtype="<i2").astype(np.float32) / 32768.0, CONFIG.board_sample_rate


@app.get("/health")
async def health():
    return {"ok": True}


@app.get("/status")
async def status():
    """/health 之外的细节放这儿 —— 这个要令牌。"""
    return {"ok": True, "light": await _exec.entity(), "auth": bool(CONFIG.api_token),
            "netease": _ne.logged_in, "llm": await _llm.available(), **_router.status()}


@app.get("/devices")
async def devices():
    """三台设备现在各自是什么状态。

    要看明白一件事：tivoli 那一栏是**影子状态**，不是设备回报的真相 ——
    设备什么都不回报。anchored_s_ago 越大，这栏越不可信；
    中间只要有人拿实体遥控器按过，它就已经错了。
    """
    return _router.status()


@app.post("/tivoli/ir/{button}")
async def tivoli_ir(button: str, times: int = 1):
    """直接发一条红外。给 M6 实测用的 —— 数源循环、试长按帧数都靠它。

    刻意不做白名单：这是排查口子，能按的键本来就写在 tivoli_ir.yaml 里，
    按错了最多是设备做了件我们没想要的事，不会有别的后果。
    """
    ok = await _tivoli.press(button, times=times)
    return {"ok": ok, "button": button, "times": times}


@app.post("/tivoli/anchor")
async def tivoli_anchor():
    """推一段静音，看能不能锚上。M6 的第一项，也是排查"设备到底开着没"的标准手段。"""
    t = _Timer()
    ok = await _tivoli.anchor()
    t.mark("anchor")
    return {"ok": ok, "shadow": _tivoli.shadow.as_dict(), "ms": t.done(),
            "meaning": "开着 + 在 WiFi 源 + 网络通" if ok else "关着，或者停在别的源上，或者掉网"}


@app.post("/tivoli/hold_frames")
async def tivoli_hold_frames(value: int):
    """改板上"长按连发多少帧"这个数。M6 扫这个值时用 —— 板上是个 template number，
    所以扫一遍是拧旋钮，不是重烧固件。"""
    try:
        await _ha.call("number", "set_value",
                       entity_id="number.tivoli_ir_hold_frames", value=int(value))
    except Exception as exc:  # noqa: BLE001
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=502)
    _tivoli.conf["hold_frames"] = int(value)
    return {"ok": True, "hold_frames": int(value)}


# 唤醒词一响就压音量，而不是等命令说完。
# 起因是实测：放着歌说「播放我喜爱的音乐」，服务端收到的是「🎼我唱唱给的算」——
# `🎼` 是 SenseVoice 的"这段是音乐"标记，也就是说麦克风那一端就已经输了，
# 规则层再聪明也没用。而唤醒词和命令词之间有 0.5~1 秒间隔，足够把音量压下去，
# 让**第一句命令**就落在安静背景上。
#
# 这件事只有我们做得到：音乐是我们自己推的，音量归我们管。
_DUCK_LEVEL = float(os.environ.get("DUCK_VOLUME", "12"))
_DUCK_SECONDS = float(os.environ.get("DUCK_SECONDS", "8"))


@app.post("/wake")
async def wake():
    """板子听到唤醒词时打这个，越快越好。

    刻意做成"发完就不管"：板子那边绝不能等这个请求的结果 ——
    它此刻正要开始收命令词，多等一毫秒都是在关键路径上。
    """
    t = _Timer()
    asyncio.create_task(_air.duck(_DUCK_LEVEL, _DUCK_SECONDS))
    return {"ok": True, "ms": t.done()}


@app.get("/music/status")
async def music_status():
    return _music.status()


# 要开机的 Tivoli 动作：这几条会触发"按 POWER + 等重新入网"，半分钟起步。
_TIVOLI_SLOW = {"radio_on", "power_on", "preset_recall", "preset_save", "station_step"}


async def _maybe_defer(parsed) -> tuple[bool, str] | None:
    """慢动作转后台，先回一句实话。

    起因是实测：Tivoli 关着时说「打开收音机」，整条命令花了 **57 秒**才返回 ——
    按 POWER、等它重新入网（28 秒）、再锚定。这期间板子一声不吭，
    用户的结论必然是"坏了"，然后再说一遍，于是又排一个 57 秒。

    把"慢"变成"说清楚要慢"，是这里唯一能做的诚实处理：设备就是要那么久开机。
    """
    if parsed.domain != "tivoli" or parsed.action not in _TIVOLI_SLOW:
        return None
    if not await _tivoli.needs_cold_start():
        return None
    asyncio.create_task(_router.execute(parsed))
    return True, "音响关着，我先给你开起来，得等半分钟"


def _commands_payload() -> dict:
    """词表 + 版本号。版本是内容的哈希，不是手工维护的序号 ——
    手工序号一定会有人忘了改，而忘了改的后果是板子上挂着一张旧表还以为是新的。"""
    cmds = intent_mod.commands_for_multinet()
    blob = json.dumps(cmds, ensure_ascii=False, sort_keys=True).encode()
    return {"version": hashlib.sha256(blob).hexdigest()[:12], "commands": cmds}


@app.get("/commands")
async def commands_endpoint():
    """板子开机时拉这张表，注册进 MultiNet。

    让服务端做词表的唯一来源：改一句命令词不用重新烧固件。
    板子拿不到（服务端没开 / 断网）时用编译进去的那份兜底，离线可用不能丢。
    """
    return _commands_payload()


@app.get("/commands/version")
async def commands_version():
    """轮询用的轻量版：几十字节，而整张表是 1.2KB。

    板子每隔几十秒问一次这个，版本变了才去拉整张表 ——
    改一条命令词从"重烧/重启板子"变成"改完 intent.py 重启服务端"。
    """
    return {"version": _commands_payload()["version"]}


@app.get("/lights")
async def lights():
    """排查用：看看 HA 里到底有哪些灯。"""
    return [{"entity_id": s["entity_id"], "state": s["state"],
             "name": s["attributes"].get("friendly_name")} for s in await _ha.lights()]


@app.post("/stt")
async def stt_endpoint(request: Request):
    audio, rate = _read_audio(await request.body())
    text = await asyncio.to_thread(STT.transcribe_array, audio, rate)
    return {"text": text}


@app.post("/tts")
async def tts_endpoint(request: Request):
    t = _Timer()
    payload = await request.json()
    text = payload.get("text", "")
    pcm = await TTS.synth_pcm_async(text)
    t.mark("synth")
    _LOG.info("TTS %r -> %.2fs 音频 [%s]", text, len(pcm) / 2 / CONFIG.board_sample_rate,
              _fmt_ms(t.marks))
    if payload.get("format") == "pcm":
        return Response(content=pcm, media_type="application/octet-stream")
    return Response(content=_pcm_to_wav(pcm, CONFIG.board_sample_rate), media_type="audio/wav")


# 追问窗口开多久。平时 0 = 用板子自己的默认（3.5s，够说一句「下一首」）。
# 但**我们主动问了问题**的时候要长得多：用户得先听完三个选项、再想一下。
# 3.5 秒实测就是"还没张嘴窗口就关了"。
_ASK_WINDOW_MS = 7000


def _followup_fields(parsed, ok: bool) -> dict:
    """追问相关的三个字段，两个 endpoint 共用（板子两条路径后面是同一段代码）。"""
    asking = _router.asking
    followup = asking or _router.wants_followup(parsed, ok)
    if followup:
        # 追问窗口期间继续压着 —— 上一轮的压制马上就要到期了
        asyncio.create_task(_air.duck(_DUCK_LEVEL, _DUCK_SECONDS))
    return {"followup": followup, "asking": asking,
            "followup_ms": _ASK_WINDOW_MS if asking else 0}


@app.post("/command")
async def command_endpoint(request: Request):
    t = _Timer()
    payload = await request.json()
    text = payload.get("text", "")
    parsed = intent_mod.parse(text, _router.ctx.rank(_router._DOMAINS))
    deferred = await _maybe_defer(parsed)
    if deferred is not None:
        ok, reply = deferred
    else:
        parsed, ok, reply = await _router.parse_and_execute(text)
    t.mark("exec")
    fu = _followup_fields(parsed, ok)
    _LOG.info("命令词 %r 意图=%s.%s(%s) 执行=%s 追问=%s  [%s]", text, parsed.domain,
              parsed.action, parsed.rule, ok, fu["followup"], _fmt_ms(t.marks))
    return JSONResponse({"text": text, "domain": parsed.domain, "action": parsed.action,
                         "rule": parsed.rule, "slots": parsed.slots,
                         "ok": ok, "reply": reply, "ms": t.done(), **fu})


@app.post("/utterance")
async def utterance_endpoint(request: Request):
    """板上 MultiNet 没认出来的那句话，整段 PCM 传上来，在这儿做开放式理解。

    和 /command 的唯一区别是前面多一步 STT，返回的 JSON 结构完全相同 ——
    板子两条路径（板上命令词 / 服务端 ASR）后面的处理因此可以是同一段代码。

    刻意不在这里回 TTS 音频：灯要在这一次往返里就动。
    回话的合成是另一件事，板子随后自己发一次 /tts，不占控灯的关键路径。
    """
    t = _Timer()
    body = await request.body()
    audio, rate = _read_audio(body)
    secs = len(audio) / rate
    # 计时器从进函数就起跑，所以这一段包含"把 body 收完"——它才是大头，
    # 解码只是一次 frombuffer。叫 recv 而不是 decode，免得看着数字去优化错的东西。
    t.mark("recv")
    # 排查用：SAVE_UTTERANCES=<目录> 就把板子传上来的原始音频存一份。
    # 板上端点检测切得对不对，只有听到它到底送了什么才说得清。
    save_dir = os.environ.get("SAVE_UTTERANCES", "").strip()
    if save_dir:
        import time
        path = os.path.join(save_dir, f"utt_{time.strftime('%H%M%S')}.wav")
        os.makedirs(save_dir, exist_ok=True)
        with open(path, "wb") as f:
            f.write(_pcm_to_wav(body, rate))
        _LOG.info("原始音频存到 %s", path)
    text = await asyncio.to_thread(STT.transcribe_array, audio, rate)
    t.mark("stt")
    parsed = intent_mod.parse(text, _router.ctx.rank(_router._DOMAINS))
    deferred = await _maybe_defer(parsed)
    if deferred is not None:
        ok, reply = deferred
    else:
        parsed, ok, reply = await _router.parse_and_execute(text)
    t.mark("exec")
    fu = _followup_fields(parsed, ok)
    # 顺带记一个"识别快过实时多少倍"：STT 慢是慢在模型还是慢在这句话太长，
    # 只看绝对毫秒数分不出来。
    _LOG.info("兜底 %.2fs 音频 -> %r 意图=%s.%s(%s) 执行=%s  [%s, %.1fx 实时]", secs, text,
              parsed.domain, parsed.action, parsed.rule, ok, _fmt_ms(t.marks),
              secs * 1000 / max(t.marks["stt"], 1))
    return JSONResponse({"text": text, "domain": parsed.domain, "action": parsed.action,
                         "rule": parsed.rule, "slots": parsed.slots,
                         "ok": ok, "reply": reply, "ms": t.done(), **fu})


@app.websocket("/voice")
async def voice_socket(ws: WebSocket):
    """板子端协议（有意做得很薄，方便在 ESP-IDF 里手写）：
      板 -> 服务端  二进制帧：16k/16bit/单声道 PCM
      板 -> 服务端  文本帧 {"event":"end"}   一句话说完
      服务端 -> 板  文本帧 {"text":…,"reply":…,"ok":…}
      服务端 -> 板  二进制帧：TTS 的 16k PCM（可能多帧）
      服务端 -> 板  文本帧 {"event":"tts_end"}
    """
    # http 中间件管不到 WebSocket 握手，这里得自己查一遍 ——
    # 否则鉴权加了半天，留着一条没上锁的控灯通道。
    if CONFIG.api_token and not hmac.compare_digest(
            ws.headers.get("authorization", ""), f"Bearer {CONFIG.api_token}"):
        await ws.close(code=1008)
        return
    await ws.accept()
    peer = ws.client.host if ws.client else "?"
    _LOG.info("板子已连接: %s", peer)
    buf = bytearray()
    try:
        while True:
            msg = await ws.receive()
            if "bytes" in msg and msg["bytes"] is not None:
                buf.extend(msg["bytes"])
                continue
            if "text" not in msg or msg["text"] is None:
                continue

            event = json.loads(msg["text"]).get("event")
            if event == "cancel":
                buf.clear()
                continue
            if event != "end":
                continue

            pcm, buf = bytes(buf), bytearray()
            secs = len(pcm) / 2 / CONFIG.board_sample_rate
            _LOG.info("收到 %.2fs 音频，开始识别", secs)

            text = await STT.transcribe_pcm_async(pcm)
            parsed, ok, reply = await _router.parse_and_execute(text)
            _LOG.info("识别=%r 意图=%s.%s 执行=%s 回话=%r", text, parsed.domain,
                      parsed.action, ok, reply)

            await ws.send_text(json.dumps(
                {"text": text, "action": parsed.action, "ok": ok, "reply": reply},
                ensure_ascii=False))

            tts_pcm = await TTS.synth_pcm_async(reply)
            chunk = 4096
            for i in range(0, len(tts_pcm), chunk):
                await ws.send_bytes(tts_pcm[i:i + chunk])
            await ws.send_text(json.dumps({"event": "tts_end"}))
    except WebSocketDisconnect:
        _LOG.info("板子断开: %s", peer)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=CONFIG.host, port=CONFIG.port, log_level="info")
