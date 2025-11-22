#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "driver/i2c.h"

#include "es8311.h"
#include "es7210.h"

namespace esphome {
namespace korvo_audio {

class KorvoAudio : public Component {
 public:
  void set_sda_pin(int pin) { this->sda_pin_ = pin; }
  void set_scl_pin(int pin) { this->scl_pin_ = pin; }
  void set_i2c_clock(int freq_hz) { this->i2c_clk_hz_ = freq_hz; }
  void set_es8311_address(uint8_t addr) { this->es8311_addr_ = addr; }
  void set_es7210_address(uint8_t addr) { this->es7210_addr_ = addr; }
  void set_speaker_sample_rate(uint32_t rate) { this->speaker_sample_rate_ = rate; }
  void set_mic_sample_rate(uint32_t rate) { this->mic_sample_rate_ = rate; }
  void set_mic_mclk_ratio(uint32_t ratio) { this->mic_mclk_ratio_ = ratio; }
  void set_speaker_volume(int volume) { this->speaker_volume_ = volume; }
  void set_pa_pin(int pin) { this->power_amp_pin_ = pin; }
  void set_pullups(bool enabled) { this->enable_pullups_ = enabled; }
  void set_mic_gain_reg(uint8_t reg) { this->mic_gain_reg_ = reg; }
  void set_mic_bias_reg(uint8_t reg) { this->mic_bias_reg_ = reg; }
  void set_mic_volume_db(int8_t volume_db) { this->mic_volume_db_ = volume_db; }
  void set_mic_tdm(bool enabled) { this->mic_tdm_enable_ = enabled; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  bool ensure_i2c_driver_();
  bool init_speaker_codec_();
  bool init_mic_codec_();
  void configure_pa_pin_();
  void log_result_(const char *target, esp_err_t err);

  int sda_pin_{1};
  int scl_pin_{2};
  int i2c_clk_hz_{400000};
  int power_amp_pin_{38};
  uint8_t es8311_addr_{0x18};
  uint8_t es7210_addr_{0x40};
  uint32_t speaker_sample_rate_{22050};
  uint32_t mic_sample_rate_{16000};
  uint32_t mic_mclk_ratio_{256};
  int speaker_volume_{85};
  int8_t mic_volume_db_{0};
  uint8_t mic_gain_reg_{ES7210_MIC_GAIN_30DB};
  uint8_t mic_bias_reg_{ES7210_MIC_BIAS_2V87};
  bool mic_tdm_enable_{true};
  bool enable_pullups_{true};

  bool i2c_ready_{false};
  es8311_handle_t es8311_{nullptr};
  es7210_dev_handle_t es7210_{nullptr};
};

}  // namespace korvo_audio
}  // namespace esphome
