#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include <string>
#include <vector>
#include <functional>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace esphome {
namespace wav_recorder_cc {

class WavRecorder : public Component {
 public:
  void set_mic(Component *mic) { this->mic_ = mic; }
  void set_sd(Component *sd) { this->sd_ = sd; }
  void set_base_path(const std::string &p) { this->base_path_ = p; }
  void set_max_seconds(int s) { this->max_seconds_ = s; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // Basic control exposed to YAML lambdas/buttons
  void start_recording();
  void stop_recording();
  bool is_recording() const { return this->recording_; }

 protected:
  void on_audio_frame_(const std::vector<uint8_t> &frame);
  bool open_file_();
  void close_file_();
  bool ensure_dir_();
  void write_wav_header_(FILE *f, uint32_t sample_rate, uint16_t bits, uint16_t channels);
  void fix_wav_sizes_(const char *path);

  Component *mic_{nullptr};
  Component *sd_{nullptr};
  std::string base_path_{"/sdcard/rec"};
  int max_seconds_{60};

  // runtime
  bool recording_{false};
  uint32_t bytes_written_{0};
  uint32_t sample_rate_{16000};
  uint16_t bits_{16};
  uint16_t channels_{1};
  std::string curr_path_;
  FILE *file_{nullptr};
};

}  // namespace wav_recorder_cc
}  // namespace esphome
