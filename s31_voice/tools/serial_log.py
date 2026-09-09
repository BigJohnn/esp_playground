#!/usr/bin/env python3
"""复位板子并抓一段串口日志。

比 `idf.py monitor` 好用的地方：非交互，抓完就退出，适合脚本化验证。
用 IDF 自带的 python 环境跑（里面有 pyserial）：
    ~/.espressif/python_env/idf6.1_py3.13_env/bin/python tools/serial_log.py 8
"""
import sys
import time

import serial

PORT = "/dev/cu.usbserial-1120"
BAUD = 115200


def main() -> None:
    args = [a for a in sys.argv[1:] if a != "--no-reset"]
    # 挂上去听、但不要重启板子。调试"说话有没有被听见"这类问题时必须的：
    # 一复位就把正在跑的状态全冲掉了。
    no_reset = "--no-reset" in sys.argv
    seconds = float(args[0]) if args else 8.0
    port = args[1] if len(args) > 1 else PORT

    with serial.Serial(port, BAUD, timeout=0.2) as ser:
        if not no_reset:
            # 经典的 ESP 自动复位时序：RTS 拉 EN，DTR 拉 BOOT。
            # 这里只复位、不进下载模式，所以 DTR 始终保持 False。
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            ser.rts = False
        ser.reset_input_buffer()

        deadline = time.time() + seconds
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()


if __name__ == "__main__":
    main()
