#!/usr/bin/env bash
# 板上命令词的声学回归：把每条命令词合成成语音、从物理扬声器放给板子听，
# 逐条核对板子识别出来的是不是同一条。
#
# 为什么需要这个：mn7_cn 有一类完全静默的失败 —— 命令词注册成功、日志一切正常、
# 发音也对，就是永远不触发（实测「最暗模式」0/9，同轮「全亮模式」2/2）。
# 光看代码和日志推不出是哪条，只能一条条念给它听。新增命令词之前先跑这个。
#
# 一条一抓，不是一轮抓到底：整轮抓的话，谁对谁错要靠时间戳去猜，
# 而识别成隔壁那条（「暖光模式」被听成「冷光模式」）恰恰是最需要抓出来的情况。
#
#   tools/mn_regress.sh                # 跑服务端当前的整张词表
#   tools/mn_regress.sh 微光模式 全亮模式  # 只验几条候选词
set -eo pipefail   # 不加 -u：bash 3.2 下空数组展开会误报 unbound variable
cd "$(dirname "$0")/.."
SERVER="${SERVER:-http://127.0.0.1:8790}"
. "$(dirname "$0")/_auth.sh"
WAKE="${WAKE:-你好小智}"
IDF_PY=~/.espressif/python_env/idf6.1_py3.13_env/bin/python

if [ $# -gt 0 ]; then
    CMDS=("$@")
else
    # 用 while read 而不是 mapfile：macOS 自带的还是 bash 3.2，没有 mapfile
    CMDS=()
    while IFS= read -r line; do CMDS+=("$line"); done < <(curl -sf "${CURL_AUTH[@]}" "$SERVER/commands" \
        | python3 -c 'import json,sys;[print(c["text"]) for c in json.load(sys.stdin)["commands"]]')
fi

VOL=$(osascript -e 'output volume of (get volume settings)')
cleanup() { osascript -e "set volume output volume $VOL" >/dev/null 2>&1 || true
            pkill -f serial_log.py 2>/dev/null || true; }
trap cleanup EXIT
osascript -e 'set volume output volume 85' >/dev/null

pass=0; wake=0; fail=()
for c in "${CMDS[@]}"; do
    pkill -f serial_log.py 2>/dev/null || true
    sleep 0.5
    LOG=$(mktemp)
    "$IDF_PY" tools/serial_log.py 14 --no-reset > "$LOG" 2>&1 &
    disown %% 2>/dev/null || true   # 免得 pkill 之后 shell 打一行 Terminated
    sleep 1.5
    tools/speak.sh 0.4 "$WAKE" 1.2 "$c" >/dev/null 2>&1
    sleep 5
    pkill -f serial_log.py 2>/dev/null || true

    # 串口是 \r\n 收行的，不剥掉 \r 的话比对永远不相等（而且打印出来看着还是对的）
    hits=$(tr -d '\r' < "$LOG" | grep -o '命令词 id=[0-9]* prob=[0-9.]* -> .*' || true)
    got=$(printf '%s' "$hits" | tail -1)
    heard=${got##*-> }
    # 窗口里出现不止一条 = 播放之外还有别的声音进来了（最常见的就是屋里有人说话）。
    # 不喊出来的话，人说的那句会被当成这次播放的结果，把一次好好的测试判成"误识别"
    # —— 这误导过一次，还差点让我去 A/B 一个根本没坏的改动。
    n_hits=$(printf '%s' "$hits" | grep -c . || true)
    if [ "$n_hits" -gt 1 ]; then
        printf '  ⚠ 这一轮听到 %s 条命令词，屋里可能有人说话，本条结果不可信：\n' "$n_hits"
        printf '%s\n' "$hits" | sed 's/^/      /'
    fi
    prob=$(printf '%s' "$got" | sed -n 's/.*prob=\([0-9.]*\).*/\1/p')
    grep -q "听到唤醒词" "$LOG" && wake=$((wake+1)) || true

    if [ "$heard" = "$c" ]; then
        pass=$((pass+1)); printf '✓ %-10s prob=%s\n' "$c" "$prob"
    elif [ -n "$heard" ]; then
        fail+=("$c → 听成了「$heard」"); printf '✗ %-10s 听成了「%s」\n' "$c" "$heard"
    else
        fail+=("$c → 完全没触发"); printf '✗ %-10s 完全没触发\n' "$c"
    fi
    rm -f "$LOG"
done

n=${#CMDS[@]}
echo
echo "唤醒 $wake/$n   命令词正确 $pass/$n"
[ ${#fail[@]} -eq 0 ] || { printf '  %s\n' "${fail[@]}"; exit 1; }
