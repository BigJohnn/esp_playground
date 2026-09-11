#!/usr/bin/env python3
"""M6：把 Tivoli 的物理常数一次量出来，写进 server/tivoli.json。

roadmap 把 M6 列成闸门 —— 它不过，FM 那条链后面全是猜的，所以 tivoli.json 里
measured=false 时执行器会直接拒绝动作，而不是瞎按。这个脚本就是过闸门的流程。

    server/.venv/bin/python tools/m6_bench.py

**会让机器发出声音**，而且要你在旁边看着屏幕回答几个问题 —— 这正是它
不能自动跑的原因：我们读不回设备的显示屏，唯一的读数器官是你的眼睛。

每一步都可以跳过（回车），跳过的项保持原值不变，最后才问要不要落盘。
"""
import argparse
import asyncio
import json
import os
import sys

import httpx

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
CONF = os.path.join(ROOT, "server", "tivoli.json")

# 设备可能出现的源。用户看屏幕报哪个，我们对上号。
SOURCES = {"wifi": ["wifi", "wi-fi", "网络", "network"],
           "bt": ["bt", "蓝牙", "bluetooth"],
           "dab": ["dab", "dab+"],
           "fm": ["fm", "调频", "收音"],
           "aux": ["aux", "line", "输入"]}


def norm_source(s: str) -> str | None:
    s = s.strip().lower()
    for key, words in SOURCES.items():
        if s == key or any(w in s for w in words):
            return key
    return None


def ask(prompt: str, default: str = "") -> str:
    tail = f" [{default}]" if default else " [回车跳过]"
    try:
        return input(f"{prompt}{tail} ").strip() or default
    except (EOFError, KeyboardInterrupt):
        print()
        raise SystemExit(130)


def yes(prompt: str) -> bool:
    return ask(f"{prompt} (y/n)", "").lower().startswith("y")


class Bench:
    def __init__(self, base: str, token: str) -> None:
        self.c = httpx.AsyncClient(
            base_url=base, timeout=40.0, trust_env=False,
            headers={"Authorization": f"Bearer {token}"} if token else {})
        self.conf = json.load(open(CONF, encoding="utf-8"))

    async def close(self):
        await self.c.aclose()

    async def anchor(self) -> bool:
        r = await self.c.post("/tivoli/anchor")
        d = r.json()
        print(f"    -> {'锚上了' if d['ok'] else '没锚上'}（{d['ms']['anchor']}ms）：{d['meaning']}")
        return bool(d["ok"])

    async def press(self, button: str, times: int = 1) -> None:
        await self.c.post(f"/tivoli/ir/{button}", params={"times": times})

    # ---------------- 各步 ----------------

    async def step_anchor(self):
        print("\n=== 1/5 锚定 ===")
        print("推 0.4 秒静音过去。成功 = 设备开着 + 停在 WiFi 源 + 网络通。")
        print("这是整套方案唯一一个既能发又能验的动作，后面每一步都建立在它上面。")
        if await self.anchor():
            print("    屏幕现在应该显示的是 WiFi / 网络源。")
            return True
        print("    没锚上。可能是关着的 —— 按一下 POWER 再试。")
        await self.press("power")
        await asyncio.sleep(3)
        if await self.anchor():
            return True
        print("    还是不行。检查：设备通电了吗？和这台 Mac 在同一个网段吗？")
        print("    AIRPLAY_PASSWORD 对吗？（在设备自己的配置页上能看到）")
        return False

    async def step_stream(self):
        print("\n=== 2/5 推流出声 ===")
        print("刚才推的是静音，只验了链路。现在验它真的能出声、能显示歌名。")
        if not yes("要现在放一小段音乐吗？"):
            return
        r = await self.c.post("/command", json={"text": "播放王菲的红豆"})
        d = r.json()
        print(f"    -> {d['reply']}")
        if not d["ok"]:
            print("    没放起来。上面这句话就是原因。")
            return
        await asyncio.sleep(12)
        print(f"    听见声音了吗？屏幕上有没有出现歌名和歌手？")
        heard = yes("    听见了？")
        shown = yes("    屏幕上显示歌名了？")
        await self.c.post("/command", json={"text": "停止播放"})
        print(f"    记录：出声={heard} 元数据={shown}")
        if heard and not shown:
            print("    （元数据没显示不影响播放，只是屏幕上看不到歌名。）")

    async def step_source_cycle(self):
        print("\n=== 3/5 数源循环 ===")
        print("SOURCE 是**循环**不是选择。想从 WiFi 走到 FM，只能数着按过去，")
        print("而本机到底有没有 DAB 决定了这个数是 1 还是 3（roadmap R3）。")
        print("下面每按一次 SOURCE，你看一眼屏幕，把它显示的源名字打给我。")
        print("认得的写法：wifi / bt / dab / fm / aux（中文也行）。看完一圈回到起点就打 done。")
        if not yes("现在开始数吗？（先确认设备是开着的、停在 WiFi 上）"):
            return
        cycle = ["wifi"]
        for i in range(1, 9):
            await self.press("source")
            await asyncio.sleep(self.conf.get("source_settle_ms", 1500) / 1000.0)
            got = ask(f"    第 {i} 次按完，屏幕显示什么源？(done=转回起点了)")
            if not got or got.lower() == "done":
                break
            key = norm_source(got)
            if key is None:
                print(f"    「{got}」我对不上号，按 wifi/bt/dab/fm/aux 里挑一个说法")
                continue
            if key in cycle:
                print(f"    {key} 已经数过了 —— 转回来了，循环长度 {len(cycle)}")
                break
            cycle.append(key)
        if len(cycle) > 1:
            self.conf["source_cycle"] = cycle
            gap = (cycle.index("fm") - cycle.index("wifi")) % len(cycle) if "fm" in cycle else None
            print(f"    源循环 = {cycle}")
            print(f"    -> 从 WiFi 到 FM 要按 SOURCE {gap} 次" if gap is not None
                  else "    -> 这一圈里没看到 FM，是不是漏数了？")

    async def step_tune_keys(self):
        print("\n=== 4/5 调谐用哪一对方向键 ===")
        print("遥控器上有 ▲▼ 和 ◀▶ 两对。FM 下哪一对是换台，看固件，不看代码。")
        if not yes("设备现在在 FM 上吗？（不在的话先切过去）"):
            return
        for keys, label in ((("up", "down"), "up_down"), (("right", "left"), "left_right")):
            await self.press(keys[0])
            await asyncio.sleep(1.5)
            if yes(f"    按了「{keys[0]}」—— 频率变了吗？"):
                self.conf["tune_keys"] = label
                print(f"    -> 调谐键 = {label}")
                await self.press(keys[1])   # 按回去
                return
        print("    两对都没反应。可能要先按 SELECT 进调谐模式，或者这机器只能搜台。")

    async def step_hold_frames(self):
        print("\n=== 5/5 长按存台要连发多少帧 ===")
        print("设备靠「同一个键连续收到多少帧」判定长按（roadmap R4）。")
        print("板子按 108ms 一帧发，所以 18 帧 ≈ 1.9 秒。从少到多试，")
        print("**第一个能触发存台提示的帧数**就是答案 —— 再多只是白等。")
        print("注意：这一步会真的改写预设位。挑一个你不心疼的位子。")
        slot = ask("    拿哪个预设位来试？(1-6)", "6")
        if not slot.isdigit() or not 1 <= int(slot) <= 6:
            print("    跳过。")
            return
        for frames in (10, 14, 18, 24, 30, 40):
            await self.c.post("/tivoli/hold_frames", params={"value": frames})
            r = await self.c.post(f"/tivoli/ir/preset_{slot}_hold")
            if r.status_code >= 400:
                print("    发不出去 —— 板子上有 Preset N Hold 这些按钮吗？（要重新烧一次）")
                return
            await asyncio.sleep(frames * 0.108 + 1.5)
            if yes(f"    {frames} 帧（≈{frames*0.108:.1f}s）—— 屏幕上出现存台提示了吗？"):
                self.conf["hold_frames"] = frames
                print(f"    -> hold_frames = {frames}")
                return
        print("    到 40 帧都没触发。可能这机器的长按走的是 NEC 重复码而不是整帧连发，")
        print("    那要改 tivoli_ir.yaml 里的 send_ir_hold。这条失败只少一个功能，不影响别的。")

    async def run(self, only: str | None):
        steps = [("anchor", self.step_anchor), ("stream", self.step_stream),
                 ("cycle", self.step_source_cycle), ("tune", self.step_tune_keys),
                 ("hold", self.step_hold_frames)]
        for name, fn in steps:
            if only and only != name:
                continue
            await fn()

        print("\n" + "=" * 60)
        print("量到的值：")
        for k in ("source_cycle", "tune_keys", "hold_frames"):
            print(f"    {k:<16} {self.conf[k]}")
        if yes("\n写进 server/tivoli.json 并把 measured 置为 true 吗？"):
            self.conf["measured"] = True
            with open(CONF, "w", encoding="utf-8") as f:
                json.dump(self.conf, f, ensure_ascii=False, indent=2)
                f.write("\n")
            print(f"写好了：{CONF}")
            print("重启服务端之后，FM 那条链就不再拒绝动作了。")
        else:
            print("没写。measured 还是 false，FM 那条链继续拒绝动作。")


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default=os.environ.get("SERVER", "http://127.0.0.1:8790"))
    ap.add_argument("--only", choices=["anchor", "stream", "cycle", "tune", "hold"],
                    help="只跑其中一步")
    args = ap.parse_args()

    token = os.environ.get("API_TOKEN", "")
    if not token:
        env = os.path.join(ROOT, "server", ".env")
        if os.path.exists(env):
            for line in open(env, encoding="utf-8"):
                if line.startswith("API_TOKEN="):
                    token = line.split("=", 1)[1].strip()

    b = Bench(args.server, token)
    try:
        await b.c.get("/health")
    except Exception:
        print(f"服务端连不上（{args.server}）。先跑 server/run.sh")
        return 1
    print("M6 —— Tivoli 物理常数实测")
    print("会让机器发出声音。挑个方便的时间，屋里最好安静一点。")
    print("每一步回车都能跳过。\n")
    try:
        await b.run(args.only)
    finally:
        await b.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
