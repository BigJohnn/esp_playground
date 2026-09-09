#!/usr/bin/env bash
# 不带麦克风的端到端冒烟测试：文本 -> 意图 -> 控灯 -> TTS
set -euo pipefail
BASE="${BASE:-http://127.0.0.1:8790}"
. "$(dirname "$0")/_auth.sh"

echo "== /health =="
curl -s "${CURL_AUTH[@]}" "$BASE/health"; echo

echo "== /lights =="
curl -s "${CURL_AUTH[@]}" "$BASE/lights"; echo

for text in "打开台灯" "调到百分之三十" "换成暖光" "关灯"; do
  echo "== /command  $text =="
  curl -s "${CURL_AUTH[@]}" -X POST "$BASE/command" -H 'content-type: application/json' \
    -d "{\"text\":\"$text\"}"; echo
  sleep 1
done

echo "== /tts =="
curl -s "${CURL_AUTH[@]}" -X POST "$BASE/tts" -H 'content-type: application/json' \
  -d '{"text":"灯已经打开了"}' -o /tmp/s31_tts.wav
ls -la /tmp/s31_tts.wav && echo "用 afplay /tmp/s31_tts.wav 听一下"
