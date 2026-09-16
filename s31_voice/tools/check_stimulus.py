#!/usr/bin/env python3
"""回归开跑之前，先验**激励本身**对不对。

起因（2026-09-16）：mn_regress 用 Kokoro 合成命令词放给板子听，而 Kokoro 把
短词的声调渲染坏了 —— 用户在旁边听见「开灯」被念成"开4灯4"，量基频证实
两音节词整个落在陈述句语调的斜坡上。也就是说那几轮回归里，板子听到的根本
不是我们以为的那个词，而"某某词不行"的结论已经被写进了词表注释，
变成了后续选词绕开的"规律"（见 README 4.1.18 的"每条至少 3 个音节"）。

判据：把合成音送回 SenseVoice，转写不回原词就说明这段激励坏了。
SenseVoice 是在真人普通话上训练的，「关灯 guān dēng」被它听成「罐凳 guàn dèng」
就是声调错了 —— 声母韵母一模一样，只有调不同。

**这把尺子有明确的局限，别把它当充分条件**：SenseVoice 带语言模型，对声调有
容错。同一轮里「开灯」通过了往返，而用户的耳朵听出它是错的。
所以这个脚本能挡住粗错，挡不住细错 —— 它是下限不是上限。

  tools/check_stimulus.py 开灯 关灯            # 验指定的几条
  tools/check_stimulus.py                      # 验服务端当前整张词表
  SPEAK_SAY_VOICE=Tingting tools/check_stimulus.py 开灯   # 验另一个激励源
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
import urllib.request

SERVER = os.environ.get("SERVER", "http://127.0.0.1:8790").rstrip("/")
SAY_VOICE = os.environ.get("SPEAK_SAY_VOICE", "").strip()


def _token() -> str:
    if os.environ.get("API_TOKEN"):
        return os.environ["API_TOKEN"]
    env = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "server", ".env")
    if os.path.exists(env):
        for line in open(env, encoding="utf-8"):
            if line.startswith("API_TOKEN="):
                return line.split("=", 1)[1].strip()
    return ""


def _post(path: str, data: bytes, ctype: str) -> bytes:
    req = urllib.request.Request(SERVER + path, data=data, method="POST")
    req.add_header("Content-Type", ctype)
    if _token():
        req.add_header("Authorization", f"Bearer {_token()}")
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


def _get(path: str) -> bytes:
    req = urllib.request.Request(SERVER + path)
    if _token():
        req.add_header("Authorization", f"Bearer {_token()}")
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.read()


def synth(text: str) -> bytes:
    """和 tools/speak.sh 走同一条合成路径，否则验的就不是真正要放的那段音。"""
    if SAY_VOICE:
        d = tempfile.mkdtemp()
        aiff, wav = os.path.join(d, "a.aiff"), os.path.join(d, "a.wav")
        subprocess.run(["say", "-v", SAY_VOICE, "-o", aiff, text],
                       check=True, capture_output=True)
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", aiff,
                        "-ar", "16000", "-ac", "1", "-c:a", "pcm_s16le", wav],
                       check=True, capture_output=True)
        return open(wav, "rb").read()
    import json
    return _post("/tts", json.dumps({"text": text}).encode(), "application/json")


_PUNCT = re.compile(r"[。，！？、,.!?\s]")


def _tones(text: str) -> str:
    """带调拼音。**比拼音不比汉字** —— 我们要抓的是声调错，不是同音字。

    「最暗」和「罪案」都是 zui4 an4，STT 挑了哪个字跟合成音的质量无关；
    而「关灯 guan1 deng1」被听成「罐凳 guan4 deng4」才是真的念错了。
    第一版比汉字，于是同音字全被报成失败，噪声大到没法用。

    留一个已知局限：**含多音字的条目这把尺子判不了**。「调暗了」被听成
    「调案了」时，pypinyin 读「调暗」是 tiao2（pinyin_fix 词典里有），
    读「调案」是 diao4（词典里没有）—— 差异来自词典命中与否，
    不是 STT 真听到的调。这类条目只能靠耳朵。
    """
    try:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "server"))
        import pinyin_fix
        from pypinyin import Style, lazy_pinyin
    except ImportError:
        # 系统 python 没装 pypinyin 就退回比汉字。功能不缺，只是同音字会误报，
        # 所以在结果里说清楚，别让人拿一份噪声大的报告当结论。
        return text + "  [按汉字比：没有 pypinyin，同音字会误报]"

    pinyin_fix.apply()
    return " ".join(lazy_pinyin(text, style=Style.TONE3, neutral_tone_with_five=True))


def main(argv: list[str]) -> int:
    import json
    words = argv[1:]
    if not words:
        words = [c["text"] for c in json.loads(_get("/commands"))["commands"]]

    src = f"macOS say -v {SAY_VOICE}" if SAY_VOICE else "服务端 Kokoro"
    print(f"激励源：{src}    判据：合成音送回 SenseVoice 能不能转写回原词")
    bad = []
    for w in words:
        wav = synth(w)
        heard = _PUNCT.sub("", json.loads(_post("/stt", wav, "audio/wav")).get("text", ""))
        ok = _tones(heard) == _tones(w)
        note = "" if heard == w else f"（{heard}，同音）" if ok else ""
        print(f"  {'✓' if ok else '✗'} {w:<10} 听回来 {heard!r} {note}")
        if not ok:
            bad.append((w, heard))
    if bad:
        print(f"\n{len(bad)} 条激励可疑 —— 板子听到的不是你以为的那个词，"
              f"这一轮的结论不可信：")
        for w, heard in bad:
            print(f"    {w} -> {heard}     {_tones(w)}  ->  {_tones(heard)}")
        print("  同音字已经自动放行（比的是带调拼音不是汉字）。"
              "剩下的都是声调真的不一样，必须换激励源或改措辞。")
        return 1
    print("\n激励全部对得上。注意这只是下限：STT 对声调有容错，细的声调错它挡不住。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
