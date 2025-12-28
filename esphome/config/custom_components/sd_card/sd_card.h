#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "sdmmc_cmd.h"
#include <string>

namespace esphome {
namespace sd_card_cc {

class SDCardMount : public Component {
 public:
  void set_pins(int clk, int cmd, int d0) { this->clk_pin_ = clk; this->cmd_pin_ = cmd; this->d0_pin_ = d0; }
  void set_bus_width(int w) { this->width_ = w; }
  void set_mount_point(const std::string &m) { this->mount_point_ = m; }
  void set_max_files(int n) { this->max_files_ = n; }
  void set_format_if_fail(bool v) { this->format_if_fail_ = v; }
  void set_alloc_unit_kb(int kb) { this->alloc_unit_kb_ = kb; }
  void set_power_pin(int pin) { this->power_pin_ = pin; }
  void set_power_active_low(bool low) { this->power_active_low_ = low; }
  void set_power_on_delay_ms(int ms) { this->power_on_delay_ms_ = ms; }
  void set_frequency_khz(int khz) { this->freq_khz_ = khz; }
  bool is_mounted() const { return this->mounted_; }
  void unmount();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE - 10.0f; }

 protected:
  void power_on_();
  void unmount_();
  bool mounting_{false};
  int clk_pin_{-1};
  int cmd_pin_{-1};
  int d0_pin_{-1};
  int width_{1};
  std::string mount_point_{"/sdcard"};
  int max_files_{5};
  bool format_if_fail_{false};
  int alloc_unit_kb_{16};
  bool mounted_{false};
  // Optional power control & tuning
  int power_pin_{-1};
  bool power_active_low_{false};
  int power_on_delay_ms_{50};
  int freq_khz_{26000};  // conservative default
  sdmmc_card_t *card_{nullptr};
};

}  // namespace sd_card_cc
}  // namespace esphome
