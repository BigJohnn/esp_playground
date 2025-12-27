# Korvo Audio（ESP32-S3-KORVO-1）优化与扩展计划

> 面向文件：esphome/config/audio.yaml
> 依据：ESP32-S3-KORVO-1 BSP 能力与引脚映射、现有 YAML 配置与目标场景（HA 语音管道）

## 1. 目标
- 降低卡顿/爆音与端到端延迟，提升稳定性。
- 对齐 BSP 引脚与默认参数，补齐易错项（如 I2S1 DIN）。
- 逐步扩展板上外设能力（环形灯、6 键、SD 录音等）。
- 保留两类配置档：低时延版、稳定版（缓冲更大）。

## 2. 现状核对（结论）
- 引脚：I2C/PA/I2S0/I2S1/LRCLK/DOUT 均对齐 BSP，麦克风 DIN=GPIO11 未在 YAML 明确（需添加）。
- 采样率：Spk=22050Hz，Mic=16000Hz（与 BSP 默认一致）。
- external_components 指向 custom_components，但当前仓库未见该目录（需确认构建环境）。
- mixer/resampler 缓冲为 500ms，稳定但时延偏大（可场景化调整）。

## 3. 立即优化项（迭代 1）
1) 明确 I2S1 麦克风 DIN 引脚
```yaml
microphone:
  - platform: korvo_tdm_mic
    id: korvo_mic
    i2s_audio_id: korvo_mic_bus
    i2s_din_pin: GPIO11   # 与 BSP: BSP_I2S1_DSIN 对齐
```

2) 降低整体时延（在稳定与时延之间折中）
```yaml
speaker:
  - platform: mixer
    id: korvo_mixer
    output_speaker: korvo_speaker
    source_speakers:
      - id: announcement_spk_mixer_input
        buffer_duration: 300ms   # 500ms -> 300ms
        timeout: 2s
      - id: media_spk_mixer_input
        buffer_duration: 200ms   # 500ms -> 200ms
        timeout: 2s

  - platform: resampler
    id: announcement_spk_resampling_input
    output_speaker: announcement_spk_mixer_input
    buffer_duration: 300ms

  - platform: resampler
    id: media_spk_resampling_input
    output_speaker: media_spk_mixer_input
```

3) 功放防爆音与音量管理
- 建议将 `speaker_volume: 100` 下调到 `70` 左右。
- 如自定义 `korvo_audio` 支持软启动参数（如 `pa_soft_start_ms`/`pa_soft_stop_ms`），开启软斜坡；否则在播放开始前预填充静音帧。
```yaml
korvo_audio:
  ...
  speaker_volume: 70
  # 若支持：
  # pa_soft_start_ms: 50
  # pa_soft_stop_ms: 50
```

4) SDK/日志配置微调（提升实时性，降低干扰）
```yaml
esp32:
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_ESP32S3_DATA_CACHE_64KB: y
      CONFIG_ESP32S3_SPIRAM_SUPPORT: y
      CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: "240"
      CONFIG_FREERTOS_UNICORE: n
      CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL: "32768"
      # 若使用 SD/FATFS：
      # CONFIG_FATFS_LONG_FILENAMES: y

logger:
  hardware_uart: USB_SERIAL_JTAG
  level: INFO   # DEBUG -> INFO，生产环境降噪
```

5) 组件目录检查
- 确认构建机存在 `custom_components` 并包含 `korvo_audio`/`korvo_tdm_mic` 等；否则调整 `external_components` 来源或将组件移入本仓库。

## 4. 功能扩展（迭代 2/3）

1) 环形灯（GPIO19，12 颗 WS2812）：语音状态指示
```yaml
light:
  - platform: esp32_rmt_led_strip
    rgb_order: GRB
    pin: GPIO19
    num_leds: 12
    name: "Korvo Ring"
    chipset: WS2812

voice_assistant:
  ...
  on_start:
    - light.turn_on:
        id: korvo_ring
        brightness: 80%
  on_listening:
    - light.addressable_set:
        id: korvo_ring
        range_from: 0
        range_to: 11
        red: 0%
        green: 100%
        blue: 0%
  on_end:
    - light.turn_off: korvo_ring
  on_error:
    - light.addressable_set:
        id: korvo_ring
        range_from: 0
        range_to: 11
        red: 100%
        green: 0%
        blue: 0%
```

2) 6 键 ADC 键盘（GPIO8，共用 ADC1_CH7）：PTT/音量/模式
- 参考 BSP 阈值（可按实测微调 ±80mV）。
```yaml
sensor:
  - platform: adc
    id: korvo_buttons_adc
    pin: GPIO8
    attenuation: 11db
    update_interval: 20ms
    filters:
      - sliding_window_moving_average:
          window_size: 5
          send_every: 1
    on_value:
      then:
        - lambda: |-
            const float mv = id(korvo_buttons_adc).state * 1000.0f; // 若单位为伏特，转毫伏
            auto in = [&](float v, float lo, float hi){ return v >= lo && v <= hi; };
            if (in(mv, 2310, 2510)) id(korvo_ptt).press();           // REC/PTT
            else if (in(mv, 1880, 2080)) {/* MODE */}
            else if (in(mv, 1560, 1760)) {/* PLAY */}
            else if (in(mv, 1010, 1210)) {/* SET */}
            else if (in(mv,  720,  920)) {/* VOLDOWN */}
            else if (in(mv,  280,  480)) {/* VOLUP */}
```

3) SD 卡（SDMMC 1-bit）：本地录音/日志
```yaml
sd_card:
  id: korvo_sd
  clk_pin: GPIO18
  cmd_pin: GPIO17
  d0_pin: GPIO16
  powerdown_pin: ""
  # width: 1 (默认)
```

4) 多麦 TDM 拓展
- 在自定义 mic 组件中暴露：通道选择（slot mask）、通道混合/平均、简单波束形成。
- 向 voice_assistant 提供更干净的近讲流；保留原始多通道供诊断/训练（可选写 SD）。

5) 声学链路增强
- 若自定义组件可用：AEC/AGC/NS 管线（AEC 需扬声器参考流）。
- 在 YAML 中增加开关与参数，便于在线调优。

6) 健康监控/自恢复
- 统计 I2S 下溢/上溢与重启次数；异常自动重建 I2S/编解码器；上报 HA 事件。

## 5. 验证方案
- 功能验证：
  - Button「Speaker Beep Test」可闻 1kHz/300ms，且无爆音/啸叫。
  - PTT 流程：开始/监听/识别/结束回调日志正确；HA 收到 STT/事件。
  - 麦克风有效输入（含静音环境下低底噪）。
- 性能验证：
  - 端到端播报延迟（TTS 触发 -> 可闻）：低时延版 < 600ms；稳定版 < 1s。
  - 连续播报与换源无明显“咔嗒”，I2S 下溢计数为 0。
- 稳定性：
  - 连续运行 24h 无异常重启；Wi-Fi 抖动下自动恢复正常音频流。

## 6. 推进计划与分档
- 迭代 1（本周）：I2S1 DIN 明确化、缓冲降至 300/200ms、音量 70、日志 INFO、组件目录确认。
- 迭代 2（下周）：环形灯、6 键 ADC、SD 卡接入；语音状态联动。
- 迭代 3：TDM 通道选择/波束形成、AEC/AGC/NS、健康监控/自恢复。
- 双配置档：
  - 低时延版：announcement 200–300ms、media 150–200ms；适合交互优先。
  - 稳定版：announcement/media 400–600ms；适合弱网/高负载场景。

## 7. 风险与回退
- custom_components 缺失导致构建失败：
  - 方案：在仓库内落地组件或改为远程组件源；新增 CI 校验路径。
- 缓冲过小导致下溢：
  - 方案：按 150ms -> 200ms -> 300ms 逐级回退；记录下溢计数门限。
- 不同 ESPHome/IDF 版本差异：
  - 方案：固化 minimal 要求（ESP-IDF >= 5.3），按需适配宏开关。
- SD/FAT 长文件名：
  - 方案：确需时开启 `CONFIG_FATFS_LONG_FILENAMES`，并评估 RAM 占用。

## 8. 验收标准
- 主路径：PTT -> STT -> TTS 全链路成功率 99%+，连续 100 次不掉链。
- 音质：无明显爆音/底噪尖峰；语音可懂度良好。
- 时延：低时延版指标满足 5. 验证方案要求。
- 可维护性：关键参数可在线调优（缓冲、音量、环灯、键位阈值）。

## 9. 待办清单（勾选项）
- [ ] 补充 `i2s_din_pin: GPIO11`
- [ ] 调整 mixer/resampler `buffer_duration`
- [ ] `speaker_volume: 70` 与（可选）PA 软启动
- [ ] logger.level -> INFO；确认 sdkconfig 选项
- [ ] 校验 `custom_components` 存在并可用
- [ ] 集成 `neopixelbus` 环形灯 + 语音状态联动
- [ ] 集成 6 键 ADC 键盘（阈值微调）
- [ ] 集成 SD 卡（若需要本地录音/缓存）
- [ ] 设计/落地 TDM 通道选择与波束形成接口
- [ ] 设计 AEC/AGC/NS 管线与开关
- [ ] 健康监控与自恢复（I2S/Codec 重建、HA 事件）
- [ ] 建立自动化验证脚本/指标采集
