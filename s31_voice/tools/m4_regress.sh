#!/usr/bin/env bash
# M4 兜底路径的声学回归：合成语音从物理扬声器放给板子听，看它走哪条路、灯有没有动。
#
# 和 mn_regress.sh 的分工：那个测"板上命令词认不认得出"，这个测"认不出的时候接不接得住"。
# 每句话跑完打印板子的日志摘要 + 灯的实际状态 —— 灯才是唯一算数的判据。
#
#   LIGHT_SCRIPT=... tools/m4_regress.sh "把灯调到最亮" "把灯关掉"
#
# 句子走命令行参数而不是 stdin：循环体里随便哪个子进程（ffmpeg 就是惯犯）
# 从 stdin 偷读一个字节，下一轮 read 拿到的就是半个 UTF-8 字符。
set -eo pipefail
cd "$(dirname "$0")/.."
PY=~/.espressif/python_env/idf6.1_py3.13_env/bin/python
LOG=$(mktemp)
LIGHT="${LIGHT_SCRIPT:?需要 LIGHT_SCRIPT 指向读灯状态的脚本}"

VOL=$(osascript -e 'output volume of (get volume settings)')
cleanup() { osascript -e "set volume output volume $VOL" >/dev/null 2>&1 || true
            pkill -f serial_log.py >/dev/null 2>&1 || true; rm -f "$LOG"; }
trap cleanup EXIT
osascript -e 'set volume output volume 85' >/dev/null

for phrase in "$@"; do
    # 标签在放音之前打：ffmpeg 播完会把终端的字符状态弄乱，
    # 之后那一行的中文会渲染成乱码（值本身是好的，只是显示坏了）。
    echo "──「$phrase」"
    pkill -f serial_log.py >/dev/null 2>&1 || true
    sleep 1
    ( $PY tools/serial_log.py 26 --no-reset > "$LOG" 2>&1 ) &
    sleep 2
    tools/speak.sh "你好小智" 1.0 "$phrase"
    sleep 16
    # || true：一句话没被识别到就是 0 行匹配，grep 退出码 1 会被 set -e 当成致命错误，
    # 于是整个回归在第一条就断掉 —— 而"没识别到"恰恰是最需要看到的结果之一。
    grep -E "sr: 听到|sr: 不在|sr: 唤醒后|sr: 命令词 id|main: 「|main: 服务端听成|耗时" "$LOG" \
        | sed 's/^I ([0-9]*) /  /' | tr -d '\r' || echo "  （日志里什么都没匹配到）"
    echo "  灯 -> $(server/.venv/bin/python "$LIGHT")"
done
