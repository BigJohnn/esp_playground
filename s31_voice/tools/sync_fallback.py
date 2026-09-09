#!/usr/bin/env python3
"""把固件里的断网兜底词表，按服务端的词表重新生成。

词表的唯一来源是 server/intent.py：改那一份不用重烧固件，板子开机会去拉。
但固件里还留着一份断网兜底的副本，改完服务端很容易忘了同步它 ——
而它只在断网时才生效，平时怎么测都测不出来。
selftest.py 的 check_firmware_fallback() 负责发现漂移，这个脚本负责修。

    server/.venv/bin/python tools/sync_fallback.py
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "server"))

from intent import commands_for_multinet  # noqa: E402

MARKER = "k_default_commands[] = {"


def main() -> None:
    cmds = commands_for_multinet()
    src = ROOT / "firmware/main/main.c"
    s = src.read_text()

    start = s.index(MARKER) + len(MARKER)
    end = s.index("};", start)
    width = max(len(c["text"]) for c in cmds)
    rows = "\n".join(
        '    {{ "{}",{} "{}" }},'.format(c["text"], " " * (width - len(c["text"])), c["phonemes"])
        for c in cmds)

    src.write_text(s[:start] + "\n" + rows + "\n" + s[end:])
    print(f"已同步 {len(cmds)} 条到 {src.relative_to(ROOT)}")
    print(rows)


if __name__ == "__main__":
    main()
