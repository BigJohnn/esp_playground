#!/usr/bin/env python3
"""Kokoro 中文音色横评：合成 -> SenseVoice 转写 -> 和原文逐字比。

为什么这样测：Kokoro-82M 的中文声调唱得不稳（尤其三声），而声调错了就会
变成同音别字 ——「把」唱成一声就是「八」，「灯开了」听着像「登台了」。
所以"转写回来还对不对"就是声调渲染质量的客观指标，不用靠耳朵。

测试集刻意用 intent.py 里真会念出口的那些 reply，而不是随便造的句子：
要挑的是"这台设备说话清不清楚"，不是"这个音色朗读散文好不好听"。

    server/.venv/bin/python tools/voice_bench.py
    server/.venv/bin/python tools/voice_bench.py zf_xiaoyi zm_yunxi   # 只比这两个
"""
from __future__ import annotations

import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "server"))

PHRASES = ["灯开了", "灯关了", "已调到最亮", "已调到最暗", "调亮了", "调暗了",
           "换成暖光", "换成冷光", "阅读模式", "夜灯模式", "切换了",
           "亮度调到百分之八十", "这个我还不会", "我没听清", "语音助手就绪"]

KOKORO_RATE = 24000


def _all_voices() -> list[str]:
    """当前权重目录里有哪些中文音色。v1.0 有 8 个，v1.1-zh 有 100 个。"""
    import tts as tts_mod

    d = tts_mod._model_dir()
    if d and (d / "voices").is_dir():
        return sorted(f.stem for f in (d / "voices").glob("z*.pt"))
    return ["zf_xiaobei", "zf_xiaoni", "zf_xiaoxiao", "zf_xiaoyi",
            "zm_yunjian", "zm_yunxi", "zm_yunxia", "zm_yunyang"]


def _clean(t: str) -> str:
    """只留下汉字和数字。SenseVoice 会附带情绪/事件标记（😔、<|HAPPY|> 之类），
    那是它的输出格式，不是发音错误，混进来会把评分算歪。"""
    return re.sub(r"[^\u4e00-\u9fff0-9]", "", t or "")


def main() -> None:
    # 有意走服务端同一条 TTS 代码路径（同一份权重、同一个注音补丁、同样的重采样），
    # 否则测出来的不是设备真会发出的声音。
    from stt import STT
    from tts import TTS

    voices = sys.argv[1:] or _all_voices()
    rows = []
    for v in voices:
        exact = hits = tot = 0
        wall0, audio_s, bad = time.time(), 0.0, []
        try:
            for p in PHRASES:
                wav, rate = TTS.synth(p, voice=v)
                audio_s += len(wav) / rate
                heard = _clean(STT.transcribe_array(wav, rate))
                hits += sum(1 for a, b in zip(p, heard) if a == b)
                tot += len(p)
                if heard == p:
                    exact += 1
                else:
                    bad.append(f"{p} -> {heard}")
        except Exception as exc:  # noqa: BLE001 - 一个音色坏掉不该让整轮白跑
            print(f"{v:13} 跳过：{type(exc).__name__} {exc}"[:160], flush=True)
            continue
        # 边跑边打：全部跑完要十几分钟，攒到最后再输出的话，中途出事就什么都没有了。
        print(f"{v:13} 整句 {exact:2d}/{len(PHRASES)}  逐字 {hits}/{tot}  "
              f"合成 {time.time() - wall0:.1f}s / 音频 {audio_s:.1f}s", flush=True)
        for b in bad:
            print(f"                {b}", flush=True)
        rows.append((exact, hits, v))

    rows.sort(reverse=True)
    print("\n=== 排名 ===")
    for exact, hits, v in rows[:10]:
        print(f"{v:13} 整句 {exact:2d}/{len(PHRASES)}  逐字 {hits}")


if __name__ == "__main__":
    main()
