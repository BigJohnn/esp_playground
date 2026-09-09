#pragma once

#include "esp_codec_dev.h"
#include "esp_err.h"

void voice_init(esp_codec_dev_handle_t codec);

/* 让服务端把 text 合成成语音，边收边从 NS4150B 放出来。
 * 整个过程里麦克风是静音的 —— 板上没有 AEC，见 sr_set_muted。 */
esp_err_t voice_say(const char *text);
