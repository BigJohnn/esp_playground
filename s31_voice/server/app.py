"""S31 Voice 服务端。

对外接口：
  GET  /health           活着没（唯一免鉴权的接口）
  GET  /commands         命令词表（板子开机拉一次，之后按版本增量刷新）
  GET  /commands/version 只回版本号，几十字节 —— 板子轮询用这个，不是上面那个
  POST /stt              裸 PCM / wav -> 文本
  POST /tts              文本 -> 16k PCM
  POST /command          文本 -> 解析意图、控灯、返回回话（板上 MultiNet 认出来后走这条）
  POST /utterance        整段 PCM -> STT -> 控灯（板上没认出来的兜底）
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
from config import CONFIG
from executor import LightExecutor
from ha import HomeAssistant
from stt import STT
from tts import TTS

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)-8s %(message)s",
    datefmt="%H:%M:%S",
)
_LOG = logging.getLogger("app")

_ha: HomeAssistant
_exec: LightExecutor


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _ha, _exec
    _ha = HomeAssistant()
    _exec = LightExecutor(_ha)
    if not CONFIG.ha_token:
        _LOG.warning("HA_TOKEN 没设，控灯会失败。HA -> 头像 -> 安全 -> 长期访问令牌")
    _disc_thread, _disc_stop = discovery.start(CONFIG.port)
    # miIO 握手保活。不这么做的话，每条隔了一分钟以上的命令都要先重握，
    # 那一下几百毫秒全落在用户等待里 —— 而控灯本来就是整条链上最慢的一段。
    warm_task = asyncio.create_task(_exec.keep_warm())
    yield
    warm_task.cancel()
    _disc_stop.set()
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
    return {"ok": True, "light": await _exec.entity(), "auth": bool(CONFIG.api_token)}


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


@app.post("/command")
async def command_endpoint(request: Request):
    t = _Timer()
    payload = await request.json()
    text = payload.get("text", "")
    parsed = intent_mod.parse(text)
    t.mark("intent")
    ok, reply = await _exec.execute(parsed)
    t.mark("exec")
    _LOG.info("命令词 %r 意图=%s(%s) 执行=%s  [%s]", text, parsed.action, parsed.rule, ok,
              _fmt_ms(t.marks))
    return JSONResponse({"text": text, "action": parsed.action, "rule": parsed.rule,
                         "ok": ok, "reply": reply, "ms": t.done()})


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
    parsed = intent_mod.parse(text)
    t.mark("intent")
    ok, reply = await _exec.execute(parsed)
    t.mark("exec")
    # 顺带记一个"识别快过实时多少倍"：STT 慢是慢在模型还是慢在这句话太长，
    # 只看绝对毫秒数分不出来。
    _LOG.info("兜底 %.2fs 音频 -> %r 意图=%s(%s) 执行=%s  [%s, %.1fx 实时]", secs, text,
              parsed.action, parsed.rule, ok, _fmt_ms(t.marks),
              secs * 1000 / max(t.marks["stt"], 1))
    return JSONResponse({"text": text, "action": parsed.action, "rule": parsed.rule,
                         "ok": ok, "reply": reply, "ms": t.done()})


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
            parsed = intent_mod.parse(text)
            ok, reply = await _exec.execute(parsed)
            _LOG.info("识别=%r 意图=%s 执行=%s 回话=%r", text, parsed.action, ok, reply)

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
