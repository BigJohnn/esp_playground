#!/usr/bin/env bash
# 把 macOS 的音频输出重新挂到 Tivoli 的 AirPlay 上。
#
# 为什么需要这个脚本：macOS 一旦切走 AirPlay（我们主动切、Tivoli 断电、或者
# 用红外把源从 WiFi 切到 FM），这条链路就断了，而且**不会自动恢复** ——
# 实测设备切回 WiFi 源之后等 30 秒也不回来。而 CoreAudio 那一层没有命令行
# 能把 AirPlay 目标选回来（SwitchAudioSource 只能在已存在的设备之间切，
# 而 AirPlay 断开之后这个设备干脆就不在列表里了）。
#
# 所以只剩 UI 脚本这一条路。需要在 系统设置 → 隐私与安全性 → 辅助功能 里
# 给运行它的那个程序打勾（注意是真正的宿主进程，不一定是"终端"）。
#
# 控制中心的声音面板里，设备名**没有暴露给辅助功能**（macOS 26.2 实测：
# AXTitle/AXDescription/AXValue 全空，只有选中状态 val=0/1）。所以没法按名字找，
# 只能逐个点、点完回头问 SwitchAudioSource 当前输出变成了谁 —— 变成 AirPlay 就是它。
set -uo pipefail

TARGET="${AIRPLAY_OUTPUT_NAME:-AirPlay}"
SWITCH="$(command -v SwitchAudioSource || echo /opt/homebrew/bin/SwitchAudioSource)"

current() { "$SWITCH" -c -t output 2>/dev/null; }

if [ "$(current)" = "$TARGET" ]; then
    echo "已经在 $TARGET 上了"
    exit 0
fi

# 打开 控制中心 -> 声音，返回设备 toggle 的个数
open_sound() {
    osascript <<'EOF' 2>/dev/null
tell application "System Events" to tell process "ControlCenter"
  -- 先确保面板是关着的。点菜单栏项是 toggle：面板本来开着的话，
  -- 这一下会把它关掉，后面全盘失败 —— 踩过一次。
  repeat 3 times
    if (count of windows) = 0 then exit repeat
    key code 53
    delay 0.4
  end repeat
  click menu bar item 5 of menu bar 1
  -- 等面板真的出现，而不是赌一个固定延时。赌固定值的时候
  -- 机器一忙就 "Can't get window 1"，而这条链路本来就够脆了。
  repeat 40 times
    delay 0.1
    if (count of windows) > 0 then exit repeat
  end repeat
  set g to UI element 1 of window 1
  repeat with i from 1 to (count of UI elements of g)
    set theId to "?"
    try
      set theId to (value of attribute "AXIdentifier" of UI element i of g) as text
    end try
    if theId is "controlcenter-volume" then
      click UI element i of g
      exit repeat
    end if
  end repeat
  -- 同理：等设备列表长出来
  set n to 0
  repeat 40 times
    delay 0.1
    try
      set n to (count of UI elements of (UI element 7 of UI element 1 of window 1))
    end try
    if n > 1 then exit repeat
  end repeat
  return n
end tell
EOF
}

click_nth() {
    osascript <<EOF 2>/dev/null
tell application "System Events" to tell process "ControlCenter"
  click UI element $1 of (UI element 7 of UI element 1 of window 1)
  delay 0.6
  key code 53
end tell
EOF
}

# 重试三次再放弃。控制中心这套 UI 有真实的竞态：面板刚关上、刚切过音频设备、
# 或者机器一忙，第一次点开就可能拿不到 window —— 而这不是权限问题，重试一下就好。
# 分清"要重试的偶发"和"要报错的权限问题"很重要，否则用户会被引去改一个没坏的设置。
N=""
for attempt in 1 2 3; do
    N="$(open_sound)"
    [[ "$N" =~ ^[0-9]+$ ]] && [ "$N" -gt 1 ] && break
    sleep 1.2
done
if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -le 1 ]; then
    echo "试了 3 次都打不开声音面板。" >&2
    echo "如果是第一次跑：系统设置 → 隐私与安全性 → 辅助功能，" >&2
    echo "把**运行这个脚本的那个程序**打勾（不一定是终端 —— 从 VS Code 的" >&2
    echo "集成终端里跑的话要勾 Visual Studio Code）。" >&2
    exit 2
fi

# 第 1 个是标题(AXHeading)，从第 2 个开始才是设备
for i in $(seq 2 "$N"); do
    click_nth "$i"
    sleep 2.5
    if [ "$(current)" = "$TARGET" ]; then
        echo "第 $i 个就是 Tivoli，已挂上 $TARGET"
        exit 0
    fi
    # 不是它，面板已经关了，重新打开再试下一个
    open_sound > /dev/null
done

echo "$((N-1)) 个输出设备都试过了，没有一个变成 $TARGET。" >&2
echo "Tivoli 可能没开机、或者不在同一网段。" >&2
exit 1
