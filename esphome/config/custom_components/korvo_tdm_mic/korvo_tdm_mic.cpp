#include "korvo_tdm_mic.h"

#include <cstring>
#include <vector>

#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal.h"
#endif

namespace esphome {
namespace korvo_tdm_mic {

static const char *const TAG = "korvo_tdm_mic";

KorvoTDMMicrophone::~KorvoTDMMicrophone() = default;

// Read block duration similar to i2s_audio microphone
static const uint32_t READ_DURATION_MS = 16;  // ~16ms per read

void KorvoTDMMicrophone::setup() {
  // Voice assistant expects 16-bit mono @ 16kHz
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 1, 16000);
}

void KorvoTDMMicrophone::dump_config() {
  ESP_LOGCONFIG(TAG, "Korvo TDM Microphone (I2S1)");
  ESP_LOGCONFIG(TAG, "  Pins: MCLK=%d, BCLK=%d, LRCLK=%d, DIN=%d", this->mclk_pin_, this->bclk_pin_, this->ws_pin_,
                this->din_pin_);
  ESP_LOGCONFIG(TAG, "  Stream: %u-bit, %u ch, %u Hz", this->audio_stream_info_.get_bits_per_sample(),
                this->audio_stream_info_.get_channels(), this->audio_stream_info_.get_sample_rate());
}

void KorvoTDMMicrophone::start() {
  if (this->state_ == microphone::STATE_RUNNING) return;

  if (!this->start_driver_()) {
    ESP_LOGE(TAG, "Failed to start I2S TDM driver");
    return;
  }
  this->stop_flag_ = false;
  xTaskCreate(KorvoTDMMicrophone::mic_task, "tdm_mic_task", 4096, this, 23, &this->task_handle_);
  if (this->task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to start mic task");
    this->stop_driver_();
    return;
  }
  this->state_ = microphone::STATE_RUNNING;
}

void KorvoTDMMicrophone::stop() {
  if (this->state_ == microphone::STATE_STOPPED) return;
  this->stop_flag_ = true;
  // Give the task some time to finish
  if (this->task_handle_ != nullptr) {
    // Wait for task to delete itself
    for (int i = 0; i < 50; i++) {  // up to ~500ms
      if (eTaskGetState(this->task_handle_) == eDeleted) break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  this->stop_driver_();
  this->state_ = microphone::STATE_STOPPED;
}

bool KorvoTDMMicrophone::start_driver_() {
  esp_err_t err;

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(this->port_, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;

  err = i2s_new_channel(&chan_cfg, NULL, &this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error creating channel: %s", esp_err_to_name(err));
    this->rx_handle_ = nullptr;
    return false;
  }

  i2s_tdm_clk_config_t clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(this->audio_stream_info_.get_sample_rate());
  i2s_tdm_slot_config_t slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_TDM_SLOT0 | I2S_TDM_SLOT1);

  i2s_tdm_gpio_config_t gpio_cfg = {
      .mclk = (gpio_num_t) this->mclk_pin_,
      .bclk = (gpio_num_t) this->bclk_pin_,
      .ws = (gpio_num_t) this->ws_pin_,
      .dout = GPIO_NUM_NC,
      .din = (gpio_num_t) this->din_pin_,
      .invert_flags = {
          .mclk_inv = false,
          .bclk_inv = false,
          .ws_inv = false,
      },
  };

  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg = clk_cfg,
      .slot_cfg = slot_cfg,
      .gpio_cfg = gpio_cfg,
  };

  err = i2s_channel_init_tdm_mode(this->rx_handle_, &tdm_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing TDM mode: %s", esp_err_to_name(err));
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
    return false;
  }

  err = i2s_channel_enable(this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error enabling channel: %s", esp_err_to_name(err));
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
    return false;
  }

  return true;
}

void KorvoTDMMicrophone::stop_driver_() {
  if (this->rx_handle_ != nullptr) {
    i2s_channel_disable(this->rx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
  }
}

void KorvoTDMMicrophone::mic_task(void *param) {
  KorvoTDMMicrophone *self = static_cast<KorvoTDMMicrophone *>(param);

  const uint32_t frames = self->audio_stream_info_.ms_to_frames(READ_DURATION_MS);
  const size_t bytes_per_sample = self->audio_stream_info_.samples_to_bytes(1);  // 2 bytes for 16-bit
  const size_t hw_bytes_per_frame = bytes_per_sample * 2;  // 16-bit per slot, 2 slots (slot0|slot1)
  const size_t hw_bytes_to_read = frames * hw_bytes_per_frame;
  const size_t mono_bytes_to_emit = frames * bytes_per_sample;

  std::vector<uint8_t> inbuf(hw_bytes_to_read);
  std::vector<uint8_t> outbuf(mono_bytes_to_emit);

  while (!self->stop_flag_) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(self->rx_handle_, inbuf.data(), inbuf.size(), &bytes_read, READ_DURATION_MS * 2);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "i2s read error: %s", esp_err_to_name(err));
      continue;
    }
    if (bytes_read == 0) {
      continue;  // timeout, try again
    }

    // Ensure bytes_read is multiple of one TDM frame (4 bytes for 16-bit stereo)
    bytes_read -= (bytes_read % hw_bytes_per_frame);
    if (bytes_read == 0) continue;

    // Downmix: take slot0 (first 16-bit sample) from each TDM frame (matches BSP using SLOT0|SLOT1)
    const size_t frames_read = bytes_read / hw_bytes_per_frame;
    outbuf.resize(frames_read * bytes_per_sample);
    for (size_t f = 0; f < frames_read; ++f) {
      const size_t in_index = f * hw_bytes_per_frame;  // slot0 LSB, slot0 MSB, slot1 LSB, slot1 MSB
      const size_t out_index = f * bytes_per_sample;
      outbuf[out_index + 0] = inbuf[in_index + 0];
      outbuf[out_index + 1] = inbuf[in_index + 1];
    }

    if (!self->mute_state_) {
      self->data_callbacks_.call(outbuf);
      // Fire taps for SD recorder or other consumers; must be non-blocking.
      for (auto &t : self->taps_) t(outbuf);
    }
  }

  self->stop_driver_();
  self->task_handle_ = nullptr;
  vTaskDelete(nullptr);
}

}  // namespace korvo_tdm_mic
}  // namespace esphome
