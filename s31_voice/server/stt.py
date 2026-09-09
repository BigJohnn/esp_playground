"""语音识别：SenseVoice-Small（FunASR）。

选它的理由：中文短指令场景下的准确率和延迟都比 Whisper 系好，模型只有 ~900MB，
M1 的 CPU 上就是远快于实时，不用为了 GPU 去折腾 Docker 透传（macOS 上也透不了）。
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time

import numpy as np

from config import CONFIG

_LOG = logging.getLogger("stt")


class SenseVoiceSTT:
    def __init__(self) -> None:
        self._model = None
        self._lock = threading.Lock()
        self._postprocess = None

    def _ensure_loaded(self) -> None:
        # 双重检查：加载很慢（首次还要下模型），但只该发生一次
        if self._model is not None:
            return
        with self._lock:
            if self._model is not None:
                return
            from funasr import AutoModel
            from funasr.utils.postprocess_utils import rich_transcription_postprocess

            t0 = time.time()
            _LOG.info("加载 %s (device=%s) …", CONFIG.stt_model, CONFIG.stt_device)
            self._model = AutoModel(
                model=CONFIG.stt_model,
                device=CONFIG.stt_device,
                disable_update=True,
                # VAD 交给板子和调用方，这里只做纯识别，省一次模型加载
                vad_model=None,
            )
            self._postprocess = rich_transcription_postprocess
            _LOG.info("STT 就绪，耗时 %.1fs", time.time() - t0)

    def transcribe_pcm(self, pcm: bytes, sample_rate: int | None = None) -> str:
        """pcm: 16-bit little-endian 单声道裸流。"""
        sample_rate = sample_rate or CONFIG.board_sample_rate
        audio = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
        return self.transcribe_array(audio, sample_rate)

    def transcribe_array(self, audio: np.ndarray, sample_rate: int) -> str:
        self._ensure_loaded()
        if audio.size == 0:
            return ""
        res = self._model.generate(
            input=audio,
            fs=sample_rate,
            language="zh",
            use_itn=True,
            ban_emo_unk=True,
        )
        if not res:
            return ""
        return self._postprocess(res[0]["text"]).strip()

    async def transcribe_pcm_async(self, pcm: bytes, sample_rate: int | None = None) -> str:
        return await asyncio.to_thread(self.transcribe_pcm, pcm, sample_rate)


STT = SenseVoiceSTT()
