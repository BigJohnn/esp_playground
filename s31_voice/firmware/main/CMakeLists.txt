# 不写 REQUIRES：main 组件默认就依赖构建里的全部组件（含 managed_components）。
# 一旦显式写了 REQUIRES 就会关掉这个默认行为，连 esp_timer / driver 都得自己列。
idf_component_register(
    SRCS "main.c" "audio_hw.c" "sr.c" "net.c" "voice.c"
    INCLUDE_DIRS "."
)
