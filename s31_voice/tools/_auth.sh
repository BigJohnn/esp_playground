# 被其它 tools/*.sh source。把 server/.env 里的 API_TOKEN 变成 curl 能用的头。
#
# 单独一个文件而不是每个脚本各写一遍：令牌的读法只有一处，
# 以后换成别的鉴权方式也只改这里。留空则 CURL_AUTH 是空数组，curl 照常跑。
_env_file="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/server/.env"
API_TOKEN="${API_TOKEN:-}"
if [ -z "$API_TOKEN" ] && [ -f "$_env_file" ]; then
    API_TOKEN=$(sed -n 's/^API_TOKEN=//p' "$_env_file" | head -1)
fi
if [ -n "$API_TOKEN" ]; then
    CURL_AUTH=(-H "Authorization: Bearer $API_TOKEN")
else
    CURL_AUTH=()
fi
