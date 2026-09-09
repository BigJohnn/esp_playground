#!/usr/bin/env bash
# 用服务端的 Kokoro TTS 合成一句话，从**内置扬声器**放出来给板子听。
#
# 为什么不用 afplay：afplay 只会走系统默认输出设备，而这台机器的默认输出被
# ToDesk 的虚拟声卡占着 —— 声音进了虚拟环回，房间里一点动静都没有，
# 表现出来就是"板子怎么都不唤醒"。ffmpeg 能直接点名物理扬声器，绕开这个坑。
#
#   tools/speak.sh "你好小智" 1.5 "打开台灯"
set -euo pipefail
cd "$(dirname "$0")/.."
SERVER="${SERVER:-http://127.0.0.1:8790}"
. "$(dirname "$0")/_auth.sh"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# 内置扬声器在 CoreAudio 里的序号（设备顺序会变，所以每次现查）
# （BSD sed 不认 \+，这里用 awk 解析更稳）
DEV=$(ffmpeg -hide_banner -f lavfi -i anullsrc -t 0.1 -f audiotoolbox -list_devices true - 2>&1 \
      | awk -F'[][]' '/MacBook Pro Speakers/ {print $(NF-1); exit}')
[ -n "$DEV" ] || { echo "找不到内置扬声器"; exit 1; }

i=0
args=("$@")
: > "$TMP/list.txt"
while [ $i -lt ${#args[@]} ]; do
    a="${args[$i]}"
    if [[ "$a" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        ffmpeg -hide_banner -loglevel error -f lavfi -i "anullsrc=r=16000:cl=mono" \
               -t "$a" -c:a pcm_s16le "$TMP/$i.wav"
    else
        curl -sf -m 60 -X POST "$SERVER/tts" "${CURL_AUTH[@]}" -H 'Content-Type: application/json' \
             -d "$(printf '{"text":"%s"}' "$a")" -o "$TMP/$i.wav"
    fi
    echo "file '$TMP/$i.wav'" >> "$TMP/list.txt"
    i=$((i+1))
done

ffmpeg -hide_banner -loglevel error -f concat -safe 0 -i "$TMP/list.txt" \
       -f audiotoolbox -audio_device_index "$DEV" -
