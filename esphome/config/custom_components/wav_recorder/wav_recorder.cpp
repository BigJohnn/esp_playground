#include "wav_recorder.h"

#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/time.h"

#include <sys/stat.h>
#include <dirent.h>

// Need korvo_tdm_mic tap
#include "esphome/components/korvo_tdm_mic/korvo_tdm_mic.h"

namespace esphome {
namespace wav_recorder_cc {

static const char *const TAG = "wav_recorder";

using esphome::korvo_tdm_mic::KorvoTDMMicrophone;

void WavRecorder::setup() {
  // Attach to mic tap if available
  auto *k = dynamic_cast<KorvoTDMMicrophone *>(this->mic_);
  if (k != nullptr) {
    k->add_tap([this](const std::vector<uint8_t> &frame) { this->on_audio_frame_(frame); });
  } else {
    ESP_LOGW(TAG, "Microphone is not KorvoTDMMicrophone - recorder disabled");
  }

  this->ensure_dir_();
}

void WavRecorder::dump_config() {
  ESP_LOGCONFIG(TAG, "WAV Recorder:");
  ESP_LOGCONFIG(TAG, "  Base path: %s", this->base_path_.c_str());
  ESP_LOGCONFIG(TAG, "  Max seconds: %d", this->max_seconds_);
}

bool WavRecorder::ensure_dir_() {
  struct stat st{};
  if (stat(this->base_path_.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) return true;
  int r = mkdir(this->base_path_.c_str(), 0777);
  if (r != 0) {
    ESP_LOGW(TAG, "mkdir(%s) failed errno=%d", this->base_path_.c_str(), (int)errno);
    return false;
  }
  return true;
}

void WavRecorder::start_recording() {
  if (this->recording_) return;
  if (!this->ensure_dir_()) return;
  if (!this->open_file_()) return;
  this->recording_ = true;
  this->bytes_written_ = 0;
  ESP_LOGI(TAG, "Recording to %s", this->curr_path_.c_str());
}

void WavRecorder::stop_recording() {
  if (!this->recording_) return;
  this->recording_ = false;
  this->close_file_();
  ESP_LOGI(TAG, "Stopped, saved %s", this->curr_path_.c_str());
}

bool WavRecorder::open_file_() {
  // Name by local time if available
  char name[64];
  auto now = time(nullptr);
  struct tm tm_now{};
  localtime_r(&now, &tm_now);
  snprintf(name, sizeof(name), "%04d%02d%02d-%02d%02d%02d.wav", tm_now.tm_year + 1900, tm_now.tm_mon + 1,
           tm_now.tm_mday, tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
  this->curr_path_ = this->base_path_ + "/" + name;

  this->file_ = fopen(this->curr_path_.c_str(), "wb");
  if (!this->file_) {
    ESP_LOGE(TAG, "open failed: %s errno=%d", this->curr_path_.c_str(), (int)errno);
    return false;
  }
  this->write_wav_header_(this->file_, this->sample_rate_, this->bits_, this->channels_);
  return true;
}

void WavRecorder::close_file_() {
  if (!this->file_) return;
  fflush(this->file_);
  ::fsync(::fileno(this->file_));
  fclose(this->file_);
  this->file_ = nullptr;
  this->fix_wav_sizes_(this->curr_path_.c_str());
}

void WavRecorder::on_audio_frame_(const std::vector<uint8_t> &frame) {
  if (!this->recording_ || !this->file_) return;
  fwrite(frame.data(), 1, frame.size(), this->file_);
  this->bytes_written_ += frame.size();
  // stop if over length
  uint32_t max_bytes = (uint32_t)((uint64_t)this->sample_rate_ * (this->bits_ / 8) * this->channels_ * this->max_seconds_);
  if (this->bytes_written_ >= max_bytes) {
    this->stop_recording();
  }
}

void WavRecorder::write_wav_header_(FILE *f, uint32_t sample_rate, uint16_t bits, uint16_t channels) {
  // Simple PCM WAV header with placeholder sizes; fix on close
  uint8_t hdr[44] = {0};
  auto put32 = [&](int off, uint32_t v) { hdr[off] = v & 0xFF; hdr[off+1] = (v>>8)&0xFF; hdr[off+2]=(v>>16)&0xFF; hdr[off+3]=(v>>24)&0xFF; };
  auto put16 = [&](int off, uint16_t v) { hdr[off] = v & 0xFF; hdr[off+1] = (v>>8)&0xFF; };
  memcpy(hdr+0, "RIFF", 4);
  put32(4, 36);  // placeholder chunk size
  memcpy(hdr+8, "WAVE", 4);
  memcpy(hdr+12, "fmt ", 4);
  put32(16, 16);           // fmt chunk size
  put16(20, 1);            // PCM
  put16(22, channels);
  put32(24, sample_rate);
  uint32_t byte_rate = sample_rate * channels * (bits / 8);
  put32(28, byte_rate);
  put16(32, channels * (bits/8)); // block align
  put16(34, bits);
  memcpy(hdr+36, "data", 4);
  put32(40, 0);            // placeholder data size
  fwrite(hdr, 1, sizeof(hdr), f);
}

void WavRecorder::fix_wav_sizes_(const char *path) {
  FILE *f = fopen(path, "r+");
  if (!f) return;
  fseek(f, 0, SEEK_END);
  long sz = ftell(f);
  if (sz < 44) { fclose(f); return; }
  uint32_t data_size = (uint32_t)(sz - 44);
  uint32_t riff_size = (uint32_t)(sz - 8);
  auto put32f = [&](long off, uint32_t v) {
    fseek(f, off, SEEK_SET);
    uint8_t b[4]; b[0]=v&0xFF; b[1]=(v>>8)&0xFF; b[2]=(v>>16)&0xFF; b[3]=(v>>24)&0xFF;
    fwrite(b, 1, 4, f);
  };
  put32f(4, riff_size);
  put32f(40, data_size);
  fflush(f);
  ::fsync(::fileno(f));
  fclose(f);
}

}  // namespace wav_recorder_cc
}  // namespace esphome

