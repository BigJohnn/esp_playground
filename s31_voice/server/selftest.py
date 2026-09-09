#!/usr/bin/env python3
"""服务端闭环自测：TTS 合成 -> 再喂给 STT 识别 -> 比对。

不依赖板子，也不依赖 Home Assistant。首次运行会下载模型（SenseVoice ~900MB，
Kokoro ~330MB），之后就快了。
"""
import pathlib
import time

import numpy as np

from intent import _auto_phonemes, commands_for_multinet, parse
from stt import STT
from tts import TTS

SENTENCES = ["打开台灯", "把亮度调到百分之三十", "换成暖光", "关灯"]


def to_py(text: str) -> str:
    """STT 的输出转成和命令词注音同一格式的拼音，好逐音节比。

    先把非汉字全去掉：SenseVoice 会带标点、情绪标记（😔）、还会把
    「百分之三十」规整成「30%」—— 那些都不是发音，混进来比对必然假阳性。
    """
    import re

    return _auto_phonemes(re.sub(r"[^\u4e00-\u9fff]", "", text or ""))


def main() -> None:
    print("=== TTS ===")
    t0 = time.time()
    wav, rate = TTS.synth(SENTENCES[0])
    print(f"首次合成（含加载模型）{time.time() - t0:.1f}s, {wav.size / rate:.2f}s 音频 @ {rate}Hz")

    print("\n=== STT 加载 ===")
    t0 = time.time()
    STT.transcribe_array(np.zeros(1600, dtype=np.float32), rate)
    print(f"加载耗时 {time.time() - t0:.1f}s")

    print("\n=== 闭环 ===")
    ok = 0
    for s in SENTENCES:
        t0 = time.time()
        wav, rate = TTS.synth(s)
        t_tts = time.time() - t0

        t0 = time.time()
        text = STT.transcribe_array(wav, rate)
        t_stt = time.time() - t0

        intent = parse(text)
        hit = "✓" if intent.action != "none" else "✗"
        ok += intent.action != "none"
        print(f"{hit} 原文={s!r:<22} 识别={text!r:<24} 意图={intent.action:<16} "
              f"tts={t_tts:.2f}s stt={t_stt:.2f}s")

    print(f"\n{ok}/{len(SENTENCES)} 条能正确落到意图上")
    check_multinet_phonemes()


def check_multinet_phonemes() -> None:
    """两道检查，各管一件事。

    (1) 高危多音字：注音选错读音是**静默失败** —— 命令词照样注册成功、日志一片正常，
        只是永远不触发。实测踩过：pypinyin 把「调到最亮」注成 "diao dao zui liang"，
        而人念的是 "tiao ..."，对着板子说十遍都没反应。
        只查一小撮智能家居语境里真会念错的字；把所有多音字都报出来没用 ——
        「灯(deng/ding)」这种谁都不会念错的会把信号淹掉。

    (2) TTS 念一遍再让 STT 听回来，比对拼音。这道**查不出**多音字：两边都用同一个
        pypinyin 转文字，读音错也会一致地错。它查的是另一件事 ——
        这条命令词经得起一次真实的"说出来再听回去"，没有被听成别的词。
    """
    # 智能家居指令里真会被注错音的字。value 是这个语境下的正确读音。
    risky = {"调": "tiao", "长": "chang", "重": "chong", "少": "shao",
             "了": "le", "空": "kong", "差": "cha", "行": "xing"}
    print("=== 高危多音字检查 ===")
    flagged = 0
    for c in commands_for_multinet():
        for ch, want in risky.items():
            if ch in c["text"] and want not in c["phonemes"].split():
                print(f"✗ {c['text']}：含「{ch}」，注音是 {c['phonemes']!r}，"
                      f"这个语境下多半该念 {want}")
                flagged += 1
    print("没有高危多音字" if not flagged else f"{flagged} 处要人工确认注音")

    print("\n=== 命令词回环校验（TTS 念 -> STT 听 -> 比对拼音）===")
    bad = 0
    for c in commands_for_multinet():
        wav, rate = TTS.synth(c["text"])
        heard = STT.transcribe_array(wav, rate)
        heard_py = to_py(heard)
        match = heard_py == c["phonemes"]
        bad += not match
        print(f"{'✓' if match else '✗'} {c['text']:<10} 注音={c['phonemes']:<24} "
              f"听回来={heard_py!r}")
    print("注音全部对得上" if not bad
          else f"{bad} 条对不上 —— 这些命令词在板子上很可能永远不触发")
    check_firmware_fallback()


def check_firmware_fallback() -> None:
    """固件里那份断网兜底词表，必须和服务端这份一致。

    两份表本来就会漂：改服务端不用重烧固件（这正是把词表放服务端的好处），
    于是很容易忘了同步固件那份 —— 而它只在断网时才会被用到，
    平时测不出来，等真断网了才发现命令词是旧的。刚才就漂过一次。
    """
    import re

    src = pathlib.Path(__file__).resolve().parent.parent / "firmware/main/main.c"
    if not src.exists():
        print("\n（找不到固件源码，跳过兜底词表比对）")
        return

    body = src.read_text()
    table = body[body.index("k_default_commands[] = {"):]
    table = table[:table.index("};")]
    fw = re.findall(r'\{\s*"([^"]+)",\s*"([^"]+)"\s*\}', table)
    server = [(c["text"], c["phonemes"]) for c in commands_for_multinet()]

    print("\n=== 固件兜底词表 vs 服务端词表 ===")
    if fw == server:
        print(f"一致（{len(fw)} 条）")
        return
    for t, p_ in server:
        if (t, p_) not in fw:
            print(f"✗ 固件里缺 / 不一致: {t} {p_!r}")
    for t, p_ in fw:
        if (t, p_) not in server:
            print(f"✗ 固件里多出来: {t} {p_!r}")



if __name__ == "__main__":
    main()
