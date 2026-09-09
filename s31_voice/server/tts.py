"""语音合成：Kokoro-82M。

选它的理由：82M 参数，M1 上准实时，中文音色可用。音箱场景里"回话快"比
"音色像真人"重要得多 —— 换 CosyVoice2 只需要替换这个文件里的 synth()。

Kokoro 输出 24kHz，板子的 ES8311 这条链路按 16kHz 走，所以出口统一重采样到 16k。
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time

import numpy as np

from config import CONFIG

_LOG = logging.getLogger("tts")

KOKORO_SAMPLE_RATE = 24000


def _model_dir():
    """本地权重目录，相对路径按本文件所在目录解析（不管服务从哪儿启动）。"""
    import pathlib

    if not CONFIG.tts_model_dir:
        return None
    d = pathlib.Path(CONFIG.tts_model_dir)
    if not d.is_absolute():
        d = pathlib.Path(__file__).resolve().parent / d
    return d if (d / "config.json").exists() else None


def _voice_ref(name: str) -> str:
    """音色名转成 KPipeline 认的东西：本地有 .pt 就给路径，否则原样给名字让它去下载。"""
    d = _model_dir()
    if d and (d / "voices" / f"{name}.pt").exists():
        return str(d / "voices" / f"{name}.pt")
    return name


def _resample_linear(x: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """线性重采样。24k->16k 是 3:2 的整数比，线性插值的失真在语音上听不出来，
    不值得为此引入 scipy/soxr 依赖。"""
    if src_rate == dst_rate or x.size == 0:
        return x
    n_out = int(round(x.size * dst_rate / src_rate))
    idx = np.linspace(0, x.size - 1, n_out, dtype=np.float64)
    return np.interp(idx, np.arange(x.size), x).astype(np.float32)


class KokoroTTS:
    def __init__(self) -> None:
        self._pipeline = None
        self._lock = threading.Lock()

    def _ensure_loaded(self) -> None:
        if self._pipeline is not None:
            return
        with self._lock:
            if self._pipeline is not None:
                return
            import pinyin_fix

            # 必须在 KPipeline 之前：misaki.zh 就是 jieba + pypinyin，
            # 补丁晚一步打就来不及了。
            pinyin_fix.apply()

            from kokoro import KModel, KPipeline

            t0 = time.time()
            _LOG.info("加载 Kokoro %s (voice=%s) …", CONFIG.tts_repo_id, CONFIG.tts_voice)
            model = None
            if _model_dir():
                d = _model_dir()
                ckpt = next(iter(sorted(d.glob("*.pth"))), None)
                if ckpt:
                    model = KModel(repo_id=CONFIG.tts_repo_id,
                                   config=str(d / "config.json"), model=str(ckpt))
            # repo_id 不能省：KPipeline 靠它决定 misaki 的 g2p 分支 ——
            # 只有非 '/Kokoro-82M' 结尾才会走 version='1.1'，也就是注音符号那套。
            # 传错了会拿 v1.1 的权重配 v1.0 的音素，出来是噪声。
            self._pipeline = KPipeline(lang_code="z", repo_id=CONFIG.tts_repo_id, model=model)
            _LOG.info("TTS 就绪，耗时 %.1fs", time.time() - t0)

    def synth(self, text: str, sample_rate: int | None = None,
              voice: str | None = None) -> tuple[np.ndarray, int]:
        """返回 (float32 [-1,1] 单声道, 采样率)。voice 只给 voice_bench 横评用。"""
        sample_rate = sample_rate or CONFIG.board_sample_rate
        self._ensure_loaded()
        if not text.strip():
            return np.zeros(0, dtype=np.float32), sample_rate

        chunks = []
        v = _voice_ref(voice or CONFIG.tts_voice)
        for _gs, _ps, audio in self._pipeline(text, voice=v, speed=CONFIG.tts_speed):
            chunks.append(np.asarray(audio, dtype=np.float32).reshape(-1))
        if not chunks:
            return np.zeros(0, dtype=np.float32), sample_rate
        wav = np.concatenate(chunks)
        return _resample_linear(wav, KOKORO_SAMPLE_RATE, sample_rate), sample_rate

    def synth_pcm(self, text: str, sample_rate: int | None = None) -> bytes:
        """返回板子能直接喂给 I2S 的 16-bit LE 裸流。"""
        wav, _ = self.synth(text, sample_rate)
        return (np.clip(wav, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()

    async def synth_pcm_async(self, text: str, sample_rate: int | None = None) -> bytes:
        return await asyncio.to_thread(self.synth_pcm, text, sample_rate)


TTS = KokoroTTS()
