# source 这个文件来进入 ESP-IDF v6.1 环境。
#
# 为什么要加 pyshim：Homebrew 的 python@3.14 的 ensurepip 是坏的，而 IDF 的
# detect_python.sh 只找 `python3`，会挑中它然后在建 venv 时失败。shim 把 `python3`
# 指向 3.13，IDF 的 python env 也就落在 idf6.1_py3.13_env 上，前后一致。
export IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf-v6.1}"
export PATH="$HOME/.espressif/pyshim:$PATH"
. "$IDF_PATH/export.sh"

# Wi-Fi 凭据不进仓库：真实值写在 sdkconfig.secret（已 gitignore），
# 用 IDF 的多文件 defaults 机制叠在 sdkconfig.defaults 上面。
# 顺序有意义 —— 后面的文件覆盖前面的。
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.secret"
