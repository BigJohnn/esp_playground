#pragma once

#include "esphome/core/component.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/core/helpers.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_tdm.h"
#include "driver/i2s_std.h"

namespace esphome {
namespace korvo_tdm_mic {

class KorvoTDMMicrophone : public Component, public microphone::Microphone {
 public:
  // Provide an out-of-line virtual destructor to anchor the vtable in this
  // component's translation unit and avoid linker "undefined vtable" issues
  // if the build system misses our .cpp in some configurations.
  ~KorvoTDMMicrophone() override;
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void start() override;
  void stop() override;

  // Allow external components (e.g., recorder) to tap raw mono PCM frames.
  // The callback runs in the mic task context; it must be non-blocking.
  void add_tap(std::function<void(const std::vector<uint8_t>&)> cb) { taps_.push_back(std::move(cb)); }

 protected:
  static void mic_task(void *param);

  bool start_driver_();
  void stop_driver_();

  // Korvo-1 BSP defaults: I2S1 for MIC, TDM mode, 16-bit, 16 kHz, slots 0|1
  const i2s_port_t port_ = I2S_NUM_1;
  const int mclk_pin_ = 20;  // GPIO20
  const int bclk_pin_ = 10;  // GPIO10
  const int ws_pin_ = 9;     // GPIO9 (LRCLK)
  const int din_pin_ = 11;   // GPIO11

  TaskHandle_t task_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  volatile bool stop_flag_{false};
  std::vector<std::function<void(const std::vector<uint8_t>&)>> taps_;
};

}  // namespace korvo_tdm_mic
}  // namespace esphome
