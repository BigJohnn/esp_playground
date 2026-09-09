# Korvo Audio（ESP32-S3-KORVO-1）优化与扩展计划

> 面向文件：esphome/config/audio.yaml
> 依据：ESP32-S3-KORVO-1 BSP 能力与引脚映射、现有 YAML 配置与目标场景（HA 语音管道）

## 1. 目标
- 降低卡顿/爆音与端到端延迟，提升稳定性。
- 对齐 BSP 引脚与默认参数，补齐易错项（如 I2S1 DIN）。
- 逐步扩展板上外设能力（环形灯、6 键、SD 录音等）。
- 保留两类配置档：低时延版、稳定版（缓冲更大）。

## 2. 现状核对（基于 esphome/config）
- 配置分为 `audio.yaml`（Korvo-1 语音节点），`demo_box3.yaml`/`livingroom.yaml` 面向 ESP32-S3-BOX-3 场景。
- 引脚：I2C/PA/I2S0/I2S1/LRCLK/DOUT 已在 YAML 对齐 BSP；麦克风 DIN=GPIO11 仍未在 YAML 明确（需补 `i2s_din_pin`）。
- 采样率：Spk=22050Hz，Mic=16000Hz；I2S/codec/重采样已统一为 22.05kHz。
- `external_components` 使用本仓库 `custom_components/`（已存在）。
- 缓冲：mixer 350/200ms，resampler 300/400ms；已低于原 500ms 但仍可进一步调优。
- SD/LED/按键：`sd_card`、WS2812 环形灯、ADC 6 键均已在 YAML 中启用。
- 语音链路：`voice_assistant` + `media_player` 已配置，且触发 HA 事件与灯效联动。
- `audio.yaml` 中 `korvo_rec` 相关调用仍在，但 `custom_component` 录音组件被注释；目前会编译失败或运行时报错，需要统一处理。

## 3. 立即优化项（迭代 1）
1) 明确 I2S1 麦克风 DIN 引脚（仍未落地）
```yaml
microphone:
  - platform: korvo_tdm_mic
    id: korvo_mic
    i2s_audio_id: korvo_mic_bus
    i2s_din_pin: GPIO11   # 与 BSP: BSP_I2S1_DSIN 对齐
```

2) 进一步降低整体时延（在稳定与时延之间折中）
```yaml
speaker:
  - platform: mixer
    id: korvo_mixer
    output_speaker: korvo_speaker
    source_speakers:
      - id: announcement_spk_mixer_input
        buffer_duration: 250ms   # 350ms -> 250ms
        timeout: 2s
      - id: media_spk_mixer_input
        buffer_duration: 180ms
        timeout: 2s

  - platform: resampler
    id: announcement_spk_resampling_input
    output_speaker: announcement_spk_mixer_input
    buffer_duration: 250ms

  - platform: resampler
    id: media_spk_resampling_input
    output_speaker: media_spk_mixer_input
```

3) 功放防爆音与音量管理
- 当前 `speaker_volume` 已为 85；如仍有爆音，可下调至 70–80 并观察。
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

5) 配置一致性检查
- `audio.yaml` 里禁用 `korvo_rec` 调用，或恢复 `wav_recorder` 组件配置（两者需一致）。

## 4. 功能扩展（迭代 2/3）

1) 环形灯（GPIO19，12 颗 WS2812）：语音状态指示 [完成]
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

2) 6 键 ADC 键盘（GPIO8，共用 ADC1_CH7）：PTT/音量/模式 [完成]
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

3) SD 卡（SDMMC 1-bit）：本地录音/日志 [已接入；WAV 录音暂未启用]
   - 自定义组件 `custom_components/sd_card/`：支持 1-bit、频率上限、内部上拉、防并发 remount，优化 VFS 内存占用
   - WAV 录音：`custom_components/wav_recorder/` 已在仓库，但 `audio.yaml` 目前未启用。
   - 语音回调 on_start/on_end 同步启停录音：`audio.yaml` 里仍保留调用，需与组件启用保持一致。

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
  - SD：Write/Read 按钮正常；`/sdcard/rec` 下生成 WAV 文件，可在 PC 播放；Remount 按钮可恢复异常。
- 性能验证：
  - 端到端播报延迟（TTS 触发 -> 可闻）：低时延版 < 600ms；稳定版 < 1s。
  - 连续播报与换源无明显“咔嗒”，I2S 下溢计数为 0。
- 稳定性：
  - 连续运行 24h 无异常重启；Wi-Fi 抖动下自动恢复正常音频流。
  - SD：频繁 remount 不触发 `ESP_ERR_NO_MEM`；挂载失败自动退避重试（已在组件侧加防抖），必要时延长 on_boot 挂载延时。

## 6. 推进计划与分档
- 迭代 1（本周）：I2S1 DIN 明确化、缓冲再降一档、音量 70–80 评估、修复 `korvo_rec` 调用一致性。
- 迭代 2（下周）：完善 HA 事件/按钮映射文档化与示例实体。
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


好的，SD 已稳定；录音/日志代码也写好。但容器在“读取配置”阶段仍报 NoneType，说明某个组件的 schema 校验在此版本路径返回了 None。为尽快推进，
  我做了两件事：

  - 已将 wav_recorder 从 Python 域改为 custom_component 方式注册（避免 Python schema 路径差异触发崩溃）。代码仍是 C++ 组件
    wav_recorder_cc::WavRecorder，按钮和 VA 回调对 id(korvo_rec) 的控制不变。位置：esphome/config/audio.yaml:467 起。
  - 给 korvo_tdm_mic 的 schema 做了版本兼容处理（存在 audio.set_stream_limits 才调用，没有就降级），避免平台 schema 自身返回 None。

  你现在遇到的 NoneType 仍然存在，接下来用最短路径把问题定位掉（2 步二分）

  请按下面顺序在容器里操作，每一步后都执行一次

  - esphome config /config/audio.yaml（只检查，不 OTA）

  步骤 A（排除录音器集成）

  - 暂时注释 audio.yaml 的 custom_component 这整个块（约 467–475 行）
  - 同时注释这两处引用它的逻辑：
      - 按钮 Start/Stop Recording（约 368–377 行）
      - voice_assistant 回调里两行 lambda（on_start 启录、on_end 停止，约 452–453 行）
  - 再跑 esphome config
      - 若此时通过，说明问题在录音器集成这块；我马上给你一版更“傻瓜式”的注册方式（不走 schema，不走 custom_component，直接在现有 sd_card 组件
        里挂一个小型 recorder 钩子，避免任何 Python 配置参与）。
      - 若还是报同样错误，继续步骤 B

  步骤 B（排除麦克风平台切换）

  - 暂时把 microphone: 恢复为之前可工作的 i2s_audio 版本（你当前文件里的 120–129 行附近的 i2s_audio 配置还在注释块里，直接改回），保留其它所
    有配置不变
  - 再跑 esphome config
      - 若此时通过，问题出在 korvo_tdm_mic 的 Python 平台加载，我会发一个完全不依赖 Python schema 的“平台内联”版本（把 TDM 驱动以
        custom_component 形式注册为 Microphone，规避平台路径）
      - 若仍失败，说明还有第三处 schema 触发（最可能是 sd_card），我会把 sd_card 同样改成 custom_component 挂载，以完全绕开 Python schema 校
        验

  为什么要这样二分

  - 这个错误是在“读配置”阶段抛出，ESPHome 在这一阶段不会告诉你具体是哪个域返回了 None，只能靠逐块排除。
  - 录音器改成 custom_component 后，理论上不会再参与 Python schema 校验；如果 A 后仍报错，优先怀疑 korvo_tdm_mic 平台路径（不同版本对 custom
    平台加载行为有差异）。

  现在你可以选择的两条路（选一个我来改）

  1. 快速上线（优先稳定）：暂时回退为 i2s_audio 麦克风，录音器也先注释掉；确认整个音频/环灯/6键/SD 正常；我离线把两处自定义组件都改成不依赖
     Python schema 的 custom_component 版本，再推一个可编译运行的补丁。
  2. 继续保留 korvo_tdm_mic：先按上面 A、B 二分出具体触发点；我针对触发点发最小变更（比如把麦克风平台改成 custom_component 注册、或把录音器改
     成挂在 sd_card 组件里的 C++ 小钩子）。

  补充

  - 你可以运行 esphome -v config /config/audio.yaml（加 -v）让日志在校验各步骤时输出更多上下文；如果能看到“正在校验的域名/平台名”，也直接贴给
    我，我能一次性修正对应 schema。

  等你回一条：“选 1/2 + 步骤 A/B 的结果”。我就按你的选择提交相应补丁，确保 OTA 能跑通；然后再把录音/日志恢复（不再依赖 Python schema），功能
  与现在设计保持一致。


  按A操作，Configuration is valid!
