"""服务端配置。敏感项走环境变量，不进仓库。"""
import os
from dataclasses import dataclass, field


def _env(key: str, default: str = "") -> str:
    return os.environ.get(key, default).strip()


@dataclass
class Config:
    # ---- Home Assistant ----
    # 长期访问令牌：HA -> 左下角用户头像 -> 安全 -> 长期访问令牌 -> 创建
    ha_url: str = field(default_factory=lambda: _env("HA_URL", "http://127.0.0.1:8123"))
    ha_token: str = field(default_factory=lambda: _env("HA_TOKEN"))
    # 目标灯实体，例如 light.myrkisel_e27。留空则启动时自动挑第一个 light.*
    light_entity: str = field(default_factory=lambda: _env("LIGHT_ENTITY"))

    # ---- 局域网直控（首选路径）----
    # 走 miIO 直连灯泡：实测 ~300ms，而经 HA + 小米云是 ~1000ms，且不依赖外网。
    # did/token 从 HA 的 xiaomi_home 存储里提取，见 tools/dump_miio_token.py
    miio_did: int = field(default_factory=lambda: int(_env("MIIO_DID", "0") or 0))
    miio_token: str = field(default_factory=lambda: _env("MIIO_TOKEN"))

    # ---- 鉴权 ----
    # 共享令牌，板子放在 sdkconfig.secret 的 CONFIG_S31_API_TOKEN 里。
    # 留空 = 不鉴权（老用法照跑）。威胁模型只是"同网段的其他人别把我的灯按着玩"，
    # 不是防拿到本机权限的攻击者，所以一个共享 bearer 就够，不上 TLS/mTLS。
    api_token: str = field(default_factory=lambda: _env("API_TOKEN"))

    # ---- STT ----
    stt_model: str = field(default_factory=lambda: _env("STT_MODEL", "iic/SenseVoiceSmall"))
    # SenseVoice-Small 在 M1 CPU 上已经远快于实时；MPS 在 funasr 上偶有算子缺失，
    # 所以默认 cpu，需要时用 STT_DEVICE=mps 打开。
    stt_device: str = field(default_factory=lambda: _env("STT_DEVICE", "cpu"))

    # ---- TTS ----
    # 用 Kokoro 的**中文专版权重** v1.1-zh，不是通用的 v1.0。这不是调优，是纠错：
    #   v1.0   英语为主，中文单音色的训练数据以「分钟」计，项目自己给中文音色
    #          打的分是 C/总体目标 D；中文用 IPA + 声调箭头表示，声调唱不稳，
    #          「灯开了」会念成「登台了」、「把」念成「八」。
    #   v1.1-zh >100 小时专业中文语料、100 位说话人，改用注音符号 + 声调数字，
    #          还会做三声变调（你好 -> ni2 hao3）。
    # 权重不在 HF 缓存里，放在 models/ 下（从 ModelScope 拉，比 HF 快 20 倍）。
    tts_repo_id: str = field(default_factory=lambda: _env(
        "TTS_REPO_ID", "hexgrad/Kokoro-82M-v1.1-zh"))
    tts_model_dir: str = field(default_factory=lambda: _env(
        "TTS_MODEL_DIR", "models/kokoro-v1_1-zh"))
    # 音色是**测出来的**，不是挑好听的：用设备真会念的那些回话逐条合成、
    # 再用 SenseVoice 转写回来逐字比对，见 tools/voice_bench.py。
    # zm_011 整句 12/15，剩下 3 处全是同音字（暗/案）和数字规整（八十/80），
    # 真实发音错误为零。同一套权重里音色差别很大，zf_003 只有 7/15。
    tts_voice: str = field(default_factory=lambda: _env("TTS_VOICE", "zm_011"))
    tts_speed: float = field(default_factory=lambda: float(_env("TTS_SPEED", "1.0")))

    # ---- 音频 ----
    # 板子侧 I2S 采样率：ES8311 单声道 16k/16bit，跟 WakeNet/AFE 的输入一致
    board_sample_rate: int = 16000

    host: str = field(default_factory=lambda: _env("HOST", "0.0.0.0"))
    port: int = field(default_factory=lambda: int(_env("PORT", "8790")))


CONFIG = Config()
