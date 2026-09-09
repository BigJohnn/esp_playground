#!/usr/bin/env python3
"""唤醒词的三项声学实测：误触发率、灵敏度（远场代理）、抗噪。

这三件事此前一件都没量过 —— README 里"真人只测过一次"、"远场和嘈杂环境没测"
说的就是这块。没有数字的时候，"唤醒不灵"只能靠体感争论。

跑法（必须用 IDF 的 python，里面才有 pyserial）：
    ~/.espressif/python_env/idf6.1_py3.13_env/bin/python tools/wake_bench.py level
    …… wake_bench.py snr
    …… wake_bench.py falsealarm --minutes 5

三种模式：
  level       固定环境，把唤醒词的电平一档档往下压，看在哪一档开始漏。
              衰减 6dB ≈ 距离翻倍（自由场反平方）。**这是乐观代理**：
              真实远场除了变小还会变混响，直达声/混响比下降是衰减模拟不出来的。
              所以这条曲线是性能上界，不是等效距离的实测。
  snr         电平固定，往里混粉红噪声，一档档降信噪比。
  falsealarm  逐句放不含唤醒词的中文，逐句归因。语料按"和唤醒词声音差多远"分三档：
              同音/仅声调不同（醒了是对的）、近音、日常句子。只有后两档算误唤醒。
  falsealarm --passive
              一个字都不放，只听房间本身。夜里能跑，而且它量的是**真实场景**的
              误触发率 —— 平时屋里本来就没人对着板子念干扰词，有的只是环境声。
              放语料那版量的是"最坏情况"，这版量的是"日常"，两个数都有用。

刻意都用数字域调幅/混音，而不是拧系统音量：可复现，且系统音量那条路
在这台机器上还要绕开 ToDesk 的虚拟声卡（见 speak.sh 的注释）。
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from dataclasses import dataclass, field

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = "/dev/cu.usbserial-1120"
BAUD = 115200
WAKE = "你好小智"

RE_WAKE = re.compile(r"听到唤醒词")
RE_LEVEL = re.compile(r"麦克风电平 RMS=(\d+) 峰值=(\d+)")
RE_CMD = re.compile(r"命令词 id=(-?\d+)")
RE_UTT = re.compile(r"不在命令词表里")


def server_url() -> str:
    return os.environ.get("SERVER", "http://127.0.0.1:8790")


def api_token() -> str:
    tok = os.environ.get("API_TOKEN", "")
    if tok:
        return tok
    path = os.path.join(ROOT, "server", ".env")
    if os.path.exists(path):
        for line in open(path, encoding="utf-8"):
            if line.startswith("API_TOKEN="):
                return line.split("=", 1)[1].strip()
    return ""


def tts(text: str, out_path: str) -> None:
    """让服务端合成一句话。用它自己的 TTS 是有意的：
    测试信号和设备平时听到的回声/回放同源，不引入第三个声音来源的差异。"""
    req = urllib.request.Request(
        server_url() + "/tts",
        data=json.dumps({"text": text}).encode(),
        headers={"Content-Type": "application/json"},
    )
    tok = api_token()
    if tok:
        req.add_header("Authorization", f"Bearer {tok}")
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    with open(out_path, "wb") as f:
        f.write(data)


def speaker_index() -> str:
    """内置扬声器在 CoreAudio 里的序号。每次现查 —— 设备顺序会变。"""
    p = subprocess.run(
        ["ffmpeg", "-hide_banner", "-f", "lavfi", "-i", "anullsrc", "-t", "0.1",
         "-f", "audiotoolbox", "-list_devices", "true", "-"],
        capture_output=True, text=True)
    for line in (p.stderr or "").splitlines():
        if "MacBook Pro Speakers" in line and "[" in line:
            return line.rsplit("[", 1)[1].split("]", 1)[0]
    raise SystemExit("找不到内置扬声器")


def build(src: str, dst: str, gain_db: float = 0.0, noise_db: float | None = None,
          lead_s: float = 0.6, tail_s: float = 0.6) -> None:
    """把源音频调幅、可选混粉红噪声，前后补静音。

    补静音是必要的：板子的 VAD 有 992ms 迟滞，紧贴着上一段声音开始会互相污染。
    """
    filters = [f"[0:a]volume={gain_db}dB[v]"]
    if noise_db is None:
        chain = "[v]"
    else:
        # 噪声铺满整段，音量相对唤醒词原始电平定；两路等权相加，不做归一化，
        # 归一化会把我们刚设好的信噪比又改回去。
        filters.append(f"anoisesrc=color=pink:amplitude=1[n0]")
        filters.append(f"[n0]volume={noise_db}dB,atrim=0:60[n]")
        filters.append("[v][n]amix=inputs=2:duration=first:normalize=0[chain]")
        chain = "[chain]"
    filters.append(f"{chain}adelay={int(lead_s*1000)}:all=1,"
                   f"apad=pad_dur={tail_s},aformat=sample_fmts=s16:sample_rates=16000[out]")
    subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", src,
         "-filter_complex", ";".join(filters), "-map", "[out]", dst],
        check=True)


def play(path: str, dev: str) -> None:
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", path,
                    "-f", "audiotoolbox", "-audio_device_index", dev, "-"],
                   check=True, stdin=subprocess.DEVNULL)


@dataclass
class Watch:
    """后台读串口，把关心的事件记下来。

    单独一条线程而不是播完再抓日志：唤醒是有时序的，必须知道它落在
    哪一次播放之内，否则前一轮的尾巴会算到后一轮头上。
    """
    lines: list[tuple[float, str]] = field(default_factory=list)
    _stop: threading.Event = field(default_factory=threading.Event)

    def start(self, port: str = PORT) -> None:
        import serial
        self._ser = serial.Serial(port, BAUD, timeout=0.2)
        self._ser.reset_input_buffer()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            buf += self._ser.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.append((time.time(), line.decode("utf-8", "replace").strip()))

    def stop(self) -> None:
        self._stop.set()
        self._t.join(timeout=2)
        self._ser.close()

    def since(self, t0: float) -> list[str]:
        return [s for ts, s in self.lines if ts >= t0]

    def mic_peak(self, t0: float, t1: float) -> int:
        """这段时间里板子自报的麦克风峰值。给电平扫描当"实际到达了多响"的刻度。"""
        peaks = [int(m.group(2)) for ts, s in self.lines if t0 <= ts <= t1
                 for m in [RE_LEVEL.search(s)] if m]
        return max(peaks) if peaks else 0


def trial(w: Watch, dev: str, wav: str, settle_s: float = 7.0) -> tuple[bool, int]:
    """放一次，等它把 6 秒命令词窗口走完，回 (有没有唤醒, 麦克风峰值)。"""
    t0 = time.time()
    play(wav, dev)
    time.sleep(settle_s)
    t1 = time.time()
    hit = any(RE_WAKE.search(s) for s in w.since(t0))
    return hit, w.mic_peak(t0, t1)


def run_sweep(mode: str, trials: int) -> None:
    dev = speaker_index()
    tmp = tempfile.mkdtemp(prefix="wakebench-")
    src = os.path.join(tmp, "wake.wav")
    print(f"合成「{WAKE}」…")
    tts(WAKE, src)

    if mode == "level":
        # 0 是当前这套摆位（约 2 米）的基准；-6dB 一档，自由场里等于距离翻倍。
        steps = [("0 dB (基准≈2m)", 0.0, None), ("-6 dB (≈4m)", -6.0, None),
                 ("-12 dB (≈8m)", -12.0, None), ("-18 dB (≈16m)", -18.0, None)]
    else:
        # 噪声电平相对唤醒词。-20dB 噪声 = 约 20dB SNR。
        steps = [("安静", 0.0, None), ("SNR≈20dB", 0.0, -20.0),
                 ("SNR≈10dB", 0.0, -10.0), ("SNR≈5dB", 0.0, -5.0),
                 ("SNR≈0dB", 0.0, 0.0)]

    w = Watch()
    w.start()
    print(f"{'档位':<18}{'唤醒':<10}{'麦克风峰值(中位)':<18}")
    try:
        for label, gain, noise in steps:
            wav = os.path.join(tmp, f"t_{label.replace(' ', '_')}.wav")
            build(src, wav, gain_db=gain, noise_db=noise)
            hits, peaks = 0, []
            for _ in range(trials):
                hit, peak = trial(w, dev, wav)
                hits += hit
                peaks.append(peak)
            peaks.sort()
            print(f"{label:<18}{f'{hits}/{trials}':<10}{peaks[len(peaks)//2]:<18}")
    finally:
        w.stop()


# 干扰语料。**按"和唤醒词的声音差多远"分类** —— 这一步不做的话数字会骗人：
# 第一版把「你好小志」「泥好小智」这类也算成干扰词，结果被叫醒了就记成误唤醒，
# 算出 157 次/小时。但「你好小志」= nǐ hǎo xiǎo zhì，和「你好小智」**逐字同音同调**；
# 唤醒词识别的是声音不是字，认出它是对的，不是错的。
# （SenseVoice 把那几条原样转写回「你好，小智。」，这才发现的。）
#
# 所以现在分三档，只有 near/normal 两档里的唤醒才算误唤醒：
HOMOPHONE = [           # 同音或仅声调不同 —— 被唤醒是**正确**行为，不计入误报
    ("你好小志", "同音同调"),
    ("泥好小智", "仅首字声调不同"),
    ("你好小知", "仅末字声调不同"),
    ("尼豪小治", "仅前两字声调不同"),
]
NEAR = [                # 末音节或语序不同 —— 这些才是真正的误唤醒
    ("你好小爱", None), ("你好小明", None), ("你好小度", None),
    ("你好小艺", None), ("你好小七", None), ("你好小猪", None),
    ("小智你好", "语序颠倒"),
]
NORMAL = [              # 日常句子，一点都不像 —— 这里出唤醒最要命
    ("今天天气不错，我们出去走走吧", None),
    ("帮我把桌子上的杯子拿过来", None),
    ("这个周末有什么安排吗", None),
    ("我想听一首轻音乐", None),
    ("冰箱里还有牛奶和鸡蛋", None),
    ("明天早上七点叫我起床", None),
    ("客厅的窗帘需要拉上了", None),
    ("电视声音有点大，调小一些", None),
    ("你知道最近的地铁站在哪里吗", None),
    ("这本书我已经看了一半了", None),
    ("小区门口新开了一家咖啡店", None),
    ("记得下班的时候买点水果回来", None),
]


def run_falsealarm(minutes: float) -> None:
    """逐句放、逐句归因。

    不再把整段语料拼起来循环播放：拼起来只能得到一个总数，
    而"哪一句把它叫醒了"才是能拿来做决定的信息 —— 日常句子叫醒它，
    和近音词叫醒它，是完全不同性质的问题。
    minutes 在这个模式下决定跑几轮（一轮 = 每句放一次）。
    """
    dev = speaker_index()
    tmp = tempfile.mkdtemp(prefix="wakebench-fa-")
    groups = [("同音/仅声调", HOMOPHONE), ("近音", NEAR), ("日常句子", NORMAL)]
    total_phrases = sum(len(g) for _, g in groups)

    print(f"合成 {total_phrases} 条语料…")
    wavs: dict[str, str] = {}
    for _, items in groups:
        for text, _note in items:
            raw = os.path.join(tmp, f"r{len(wavs)}.wav")
            tts(text, raw)
            out = os.path.join(tmp, f"p{len(wavs)}.wav")
            build(raw, out, lead_s=0.4, tail_s=0.6)
            wavs[text] = out

    # 一句约 9 秒（放 + 等 6 秒命令词窗口走完）
    rounds = max(1, int(minutes * 60 / (total_phrases * 9) + 0.5))
    print(f"每句放 {rounds} 遍，共约 {total_phrases * rounds * 9 / 60:.1f} 分钟\n")

    w = Watch()
    w.start()
    t0 = time.time()
    hits: dict[str, int] = {}
    try:
        for label, items in groups:
            print(f"── {label}")
            for text, note in items:
                n = 0
                for _ in range(rounds):
                    hit, _peak = trial(w, dev, wavs[text])
                    n += hit
                hits[text] = n
                mark = "醒" if n else "·"
                tail = f"   （{note}）" if note else ""
                print(f"  {mark} {text:<16} {n}/{rounds}{tail}")
            print()
    finally:
        elapsed = time.time() - t0
        w.stop()

    def count(items):
        return sum(hits.get(t, 0) for t, _ in items), len(items) * rounds

    h_hit, h_n = count(HOMOPHONE)
    n_hit, n_n = count(NEAR)
    o_hit, o_n = count(NORMAL)
    print(f"{elapsed/60:.1f} 分钟")
    print(f"  同音/仅声调  {h_hit}/{h_n}   <- 醒了是对的，不算误报")
    print(f"  近音词      {n_hit}/{n_n}   <- 误唤醒")
    print(f"  日常句子    {o_hit}/{o_n}   <- 误唤醒，且最要命")
    print(f"  真实误唤醒率 {(n_hit + o_hit)}/{n_n + o_n}")
    lines = w.since(t0)
    cmds = [s for s in lines if RE_CMD.search(s)]
    print(f"  其中误识别成命令词 {len(cmds)} 次   <- 只有这些会真的动灯")
    for s in cmds:
        print("   ", s)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["level", "snr", "falsealarm"])
    ap.add_argument("--trials", type=int, default=8)
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--passive", action="store_true",
                    help="falsealarm 专用：不放任何声音，只听环境。夜里能跑。")
    a = ap.parse_args()
    if a.mode == "falsealarm":
        run_passive(a.minutes) if a.passive else run_falsealarm(a.minutes)
    else:
        run_sweep(a.mode, a.trials)


if __name__ == "__main__":
    main()
