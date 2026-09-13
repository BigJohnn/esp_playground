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

    @staticmethod
    def unreliable(text: str) -> str | None:
        """这段识别结果可不可信。返回不可信的理由，可信就返回 None。

        SenseVoice 除了文字还会吐**音频事件标记**，rich_transcription_postprocess
        把它们转成 emoji：🎼=音乐、😀/😡 等=情绪、👏=掌声…… 其中 🎼 是
        "我认为这段是音乐不是说话"，也就是模型自己在说"别太信我下面这几个字"。

        在此之前这个信号被整个扔掉了：实测板子送上来一段被截断的录音，
        服务端听成「🎼空调。」，我们照单全收 —— 拿它去过意图层、过拼音层，
        再花 2.6 秒问 LLM，最后回一句"这个我还不会"。模型第一个字就告诉我们
        这段不可信了，后面那几步全是白做的。

        只在**文字很短**的时候才认这个标记。一段音乐里夹着一句听清了的话
        （"🎼打开台灯"）是完全可能的，那种情况该照常执行 —— 标记说的是
        "这段音频里有音乐"，不是"这几个字是错的"。字少才说明确实没听清什么。

        情绪标记（😊😔😡…）不算，见 _NON_SPEECH_TAGS 那段。
        """
        if not text:
            return "空的"
        body = "".join(ch for ch in text if not _is_event_tag(ch))
        body = body.strip(" 。，,.!?！？")
        if body != text.strip(" 。，,.!?！？") and len(body) <= 3:
            return f"SenseVoice 把这段标成了非语音（{text!r}），而且只听出 {len(body)} 个字"
        return None

    async def transcribe_pcm_async(self, pcm: bytes, sample_rate: int | None = None) -> str:
        return await asyncio.to_thread(self.transcribe_pcm, pcm, sample_rate)


# SenseVoice 吐的 emoji 分两类，含义**正好相反**，不能一把抓：
#
#   情绪标记  😊😔😡😰🤢😮  —— "这是说话，而且带着某种情绪"。
#             它反而是"确实听到人声"的证据。
#   事件标记  🎼👏🤧😷😭    —— "这段是音乐/掌声/咳嗽"，也就是**不是说话**。
#
# 第一版按 Unicode 区段一刀切，理由是"funasr 的标记表会随版本变，
# 写死清单会在某次升级后悄悄失效"。那个担心本身没错，但代价更大：
# 「😊好的」被判成不可信 —— 而「好的」正是回答问题时最常说的一句。
# 语义分歧大于维护成本，所以这里只列真正表示"非语音"的那几个。
_NON_SPEECH_TAGS = "🎼👏🤧😷😭"


def _is_event_tag(ch: str) -> bool:
    return ch in _NON_SPEECH_TAGS


STT = SenseVoiceSTT()
