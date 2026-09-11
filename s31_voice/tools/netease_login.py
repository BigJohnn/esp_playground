#!/usr/bin/env python3
"""网易云扫码登录。跑一次，cookie 落到 server/.netease_cookie（已 gitignore）。

    server/.venv/bin/python tools/netease_login.py

二维码直接画在终端里，用网易云 App 扫。cookie 里的 MUSIC_U 是长期有效的，
但会过期 —— 过期的表现是"播放我的收藏"回一句"登录过期了，重新扫一次码"，
那时候再跑一遍这个脚本就行。
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "server"))

import netease  # noqa: E402


def show_qr(url: str) -> None:
    try:
        import qrcode
    except ImportError:
        print(f"没装 qrcode 模块，自己找个二维码工具编这个链接：\n  {url}")
        return
    qr = qrcode.QRCode(border=1)
    qr.add_data(url)
    qr.make(fit=True)
    # 用半格块把二维码压到一半高度，否则终端里放不下
    m = qr.get_matrix()
    if len(m) % 2:
        m.append([False] * len(m[0]))
    for y in range(0, len(m), 2):
        line = "".join("█" if not m[y][x] and not m[y + 1][x] else
                       "▀" if not m[y][x] else
                       "▄" if not m[y + 1][x] else " "
                       for x in range(len(m[0])))
        print(line)


async def main() -> int:
    client = netease.login_client()
    try:
        key, url = await netease.qr_login_start(client)
        print("拿网易云 App 扫这个码：\n")
        show_qr(url)
        print(f"\n（扫不了就手动打开：{url}）\n")

        states = {801: "等你扫", 802: "扫上了，在手机上点确认"}
        last = None
        for _ in range(150):          # 最多等 5 分钟
            await asyncio.sleep(2)
            code, cookie = await netease.qr_login_poll(client, key)
            if code == 803:
                ne = netease.Netease()
                ne.save_cookie(cookie)
                ne.reset()
                acc = await ne.account()
                await ne.close()
                who = acc.get("nickname") if acc else "?"
                print(f"\n登录成功：{who}（uid {acc['uid'] if acc else '?'}）")
                print(f"cookie 写到 {ne.cookie_path}（权限 600，已在 .gitignore 里）")
                return 0
            if code == 800:
                print("\n二维码过期了，重跑一次")
                return 1
            if code != last:
                print(f"  … {states.get(code, code)}")
                last = code
        print("\n等超时了")
        return 1
    finally:
        await client.aclose()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
