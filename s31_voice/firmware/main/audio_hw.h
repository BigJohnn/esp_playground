#pragma once

#include <stddef.h>
#include "esp_err.h"
#include "esp_codec_dev.h"

/* 初始化 I2C + I2S + ES8311，返回可读可写的 codec 句柄。 */
esp_err_t audio_hw_init(esp_codec_dev_handle_t *out_dev);

/* 扫描 I2C 总线并把发现的地址打到日志上 —— bring-up 时先确认 ES8311 在不在 (0x18/0x19)。 */
esp_err_t audio_hw_i2c_scan(void);

/* 音量 0~100；输入增益单位 dB。 */
esp_err_t audio_hw_set_volume(esp_codec_dev_handle_t dev, int vol_pct);
esp_err_t audio_hw_set_mic_gain(esp_codec_dev_handle_t dev, float gain_db);

/* Bring-up 诊断：打印 PA 引脚电平 + ES8311 全部寄存器，再用满音量放 1kHz 正弦。
 * "写进 codec 成功"不等于"喇叭响了"，这条用来把两者分开。 */
esp_err_t audio_hw_diag(esp_codec_dev_handle_t dev);

/* 放一段正弦。I2S 是双声道的，这里负责把同一路数据写进两个槽 ——
 * ES8311 只有一路 DAC，哪个槽被它拿去无所谓，两个都写就不用猜。 */
esp_err_t audio_hw_play_tone(esp_codec_dev_handle_t dev, int freq_hz, int ms, int amplitude);

/* 把单声道 PCM 复制成双声道再写。返回写进去的单声道字节数。 */
esp_err_t audio_hw_write_mono(esp_codec_dev_handle_t dev, const void *pcm, size_t bytes);
