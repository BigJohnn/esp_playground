#!/usr/bin/env bash
# 拉 Kokoro 的中文专版权重 v1.1-zh。
#
# 为什么不是默认的 v1.0：v1.0 是英语模型，中文是顺带加的 —— 项目自己的 VOICES.md
# 把 8 个中文音色全部标成总评 C / 目标质量 D，单音色训练数据以「分钟」计。
# 它把中文表示成 IPA + 声调箭头，声调唱不稳：「灯开了」念成「登台了」、「把」念成「八」。
# v1.1-zh 是 >100 小时专业中文语料、100 位说话人，改用注音符号 + 声调数字，
# 还会做三声变调。实测（tools/voice_bench.py，用设备真会念的回话逐字比对）：
#   v1.0 最好的 zm_yunxi   11/15；默认的 zf_xiaobei 只有 5/15
#   v1.1-zh 的 zm_011      12/15，且剩下 3 处全是同音字/数字规整，发音零错
#
# 走 ModelScope 而不是 HuggingFace：实测 9.8MB/s vs 0.4MB/s，312MB 下 5 秒 vs 12 分钟。
set -euo pipefail
cd "$(dirname "$0")/.."
D=server/models/kokoro-v1_1-zh
MS="https://modelscope.cn/api/v1/models/hexgrad/Kokoro-82M-v1.1-zh/repo?Revision=master&FilePath="
mkdir -p "$D/voices"

echo "拉权重 (312MB) …"
curl -fL --max-time 600 -o "$D/kokoro-v1_1-zh.pth" "${MS}kokoro-v1_1-zh.pth"
curl -fL --max-time 60  -o "$D/config.json"        "${MS}config.json"

echo "拉音色 …"
curl -s --max-time 60 \
  "https://modelscope.cn/api/v1/models/hexgrad/Kokoro-82M-v1.1-zh/repo/files?Revision=master&Root=voices" \
  | python3 -c "import json,sys; print('\n'.join(f['Path'] for f in json.load(sys.stdin)['Data']['Files'] if f['Path'].endswith('.pt')))" \
  | while read -r p; do
        curl -fsL --max-time 60 -o "$D/voices/$(basename "$p")" "${MS}${p}" || echo "跳过 $p"
    done

echo "完成：$(ls "$D/voices" | wc -l | tr -d ' ') 个音色"
