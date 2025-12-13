#include "korvo_audio.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace korvo_audio {

static const char *const TAG = "korvo_audio";

void KorvoAudio::setup() {
  ESP_LOGCONFIG(TAG, "初始化 Korvo-1 音频编解码器...");
  this->configure_pa_pin_();

  if (!this->ensure_i2c_driver_()) {
    this->mark_failed();
    return;
  }

  if (!this->init_speaker_codec_()) {
    this->status_set_warning();
  }
  if (!this->init_mic_codec_()) {
    this->status_set_warning();
  }
}

void KorvoAudio::dump_config() {
  ESP_LOGCONFIG(TAG, "Korvo-1 音频硬件初始化");
  ESP_LOGCONFIG(TAG, "  SDA pin: GPIO%d", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL pin: GPIO%d", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  I2C clock: %d Hz", this->i2c_clk_hz_);
  ESP_LOGCONFIG(TAG, "  ES8311 addr: 0x%02X", this->es8311_addr_);
  ESP_LOGCONFIG(TAG, "  ES7210 addr: 0x%02X", this->es7210_addr_);
  ESP_LOGCONFIG(TAG, "  Speaker sample rate: %" PRIu32 " Hz", this->speaker_sample_rate_);
  ESP_LOGCONFIG(TAG, "  Mic sample rate: %" PRIu32 " Hz", this->mic_sample_rate_);
  ESP_LOGCONFIG(TAG, "  Mic MCLK ratio: %" PRIu32, this->mic_mclk_ratio_);
  ESP_LOGCONFIG(TAG, "  Mic gain reg: 0x%02X  bias: 0x%02X", this->mic_gain_reg_, this->mic_bias_reg_);
  ESP_LOGCONFIG(TAG, "  Mic TDM mode: %s", this->mic_tdm_enable_ ? "enabled" : "disabled");
}

bool KorvoAudio::ensure_i2c_driver_() {
  if (this->i2c_ready_) {
    return true;
  }

  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = static_cast<gpio_num_t>(this->sda_pin_);
  conf.scl_io_num = static_cast<gpio_num_t>(this->scl_pin_);
  conf.sda_pullup_en = this->enable_pullups_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  conf.scl_pullup_en = this->enable_pullups_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = this->i2c_clk_hz_;

  esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "i2c_param_config 失败: %s", esp_err_to_name(err));
    return false;
  }

  err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(err));
    return false;
  }

  this->i2c_ready_ = true;
  return true;
}

bool KorvoAudio::init_speaker_codec_() {
  if (this->es8311_ != nullptr) {
    return true;
  }

  this->es8311_ = es8311_create(I2C_NUM_0, this->es8311_addr_);
  if (this->es8311_ == nullptr) {
    ESP_LOGE(TAG, "创建 ES8311 句柄失败");
    return false;
  }

  es8311_clock_config_t clk_cfg = {};
  clk_cfg.mclk_inverted = false;
  clk_cfg.sclk_inverted = false;
  // clk_cfg.mclk_from_mclk_pin = true;
  // clk_cfg.mclk_frequency = static_cast<int>(this->speaker_sample_rate_ * 256);
  clk_cfg.mclk_from_mclk_pin = false;
  clk_cfg.mclk_frequency = 0;
  clk_cfg.sample_frequency = static_cast<int>(this->speaker_sample_rate_);

  esp_err_t err = es8311_init(this->es8311_, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ES8311 初始化失败: %s", esp_err_to_name(err));
    return false;
  }

  err = es8311_voice_volume_set(this->es8311_, this->speaker_volume_, nullptr);
  this->log_result_("ES8311 volume", err);
  err = es8311_voice_mute(this->es8311_, false);
  this->log_result_("ES8311 mute", err);
  return true;
}

bool KorvoAudio::init_mic_codec_() {
  if (this->es7210_ != nullptr) {
    return true;
  }

  es7210_i2c_config_t cfg = {
      .i2c_port = I2C_NUM_0,
      .i2c_addr = this->es7210_addr_,
  };

  esp_err_t err = es7210_new_codec(&cfg, &this->es7210_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "创建 ES7210 失败: %s", esp_err_to_name(err));
    return false;
  }

  es7210_codec_config_t codec = {};
  codec.sample_rate_hz = this->mic_sample_rate_;
  codec.mclk_ratio = this->mic_mclk_ratio_;
  codec.i2s_format = ES7210_I2S_FMT_I2S;
  codec.bit_width = ES7210_I2S_BITS_32B;
  codec.mic_bias = static_cast<es7210_mic_bias_t>(this->mic_bias_reg_);
  codec.mic_gain = static_cast<es7210_mic_gain_t>(this->mic_gain_reg_);
  codec.flags.tdm_enable = this->mic_tdm_enable_;

  err = es7210_config_codec(this->es7210_, &codec);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ES7210 配置失败: %s", esp_err_to_name(err));
    return false;
  }

  err = es7210_config_volume(this->es7210_, this->mic_volume_db_);
  this->log_result_("ES7210 volume", err);
  return true;
}

void KorvoAudio::configure_pa_pin_() {
  if (this->power_amp_pin_ < 0) {
    return;
  }
  gpio_config_t cfg = {};
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pin_bit_mask = 1ULL << this->power_amp_pin_;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&cfg);
  gpio_set_level(static_cast<gpio_num_t>(this->power_amp_pin_), 1);
}

void KorvoAudio::log_result_(const char *target, esp_err_t err) {
  if (err == ESP_OK) {
    ESP_LOGD(TAG, "%s 成功", target);
  } else {
    ESP_LOGW(TAG, "%s 失败: %s", target, esp_err_to_name(err));
  }
}

}  // namespace korvo_audio
}  // namespace esphome
