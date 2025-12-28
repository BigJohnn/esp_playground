#include "sd_card.h"
#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace sd_card_cc {

static const char *const TAG = "sd_card";

void SDCardMount::setup() {
  if (this->mounting_) {
    ESP_LOGW(TAG, "Mount attempt ignored (already mounting)");
    return;
  }
  this->mounting_ = true;
  ESP_LOGI(TAG, "Mounting SD card at %s (CLK=%d CMD=%d D0=%d, width=%d)", this->mount_point_.c_str(), this->clk_pin_, this->cmd_pin_, this->d0_pin_, this->width_);

  // Basic internal pull-ups on CMD and D0 to help boards without strong external pull-ups
  gpio_set_pull_mode((gpio_num_t) this->cmd_pin_, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t) this->d0_pin_, GPIO_PULLUP_ONLY);

  // Optional power control
  this->power_on_();

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  // Some boards (ESP32-S3) route the slot to SLOT_1; force it for clarity.
  host.slot = SDMMC_HOST_SLOT_1;
  // Force 1-bit if requested
  if (this->width_ == 1) {
    host.flags = (host.flags & ~SDMMC_HOST_FLAG_4BIT) | SDMMC_HOST_FLAG_1BIT;
  }
  // Optionally cap the frequency (IDF 5.x has .max_freq_khz)
  if (this->freq_khz_ > 0) {
    host.max_freq_khz = this->freq_khz_;
  }

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = (this->width_ == 4) ? 4 : 1;
  slot_config.clk = (gpio_num_t) this->clk_pin_;
  slot_config.cmd = (gpio_num_t) this->cmd_pin_;
  slot_config.d0  = (gpio_num_t) this->d0_pin_;
  slot_config.gpio_cd = (gpio_num_t) -1;  // no card-detect pin
  slot_config.gpio_wp = (gpio_num_t) -1;  // no write-protect pin

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = this->format_if_fail_;
  mount_config.max_files = this->max_files_;
  mount_config.allocation_unit_size = this->alloc_unit_kb_ * 1024;
  // Reduce RAM pressure by avoiding full FAT cache when possible
  mount_config.use_one_fat = true;
  mount_config.disk_status_check_enable = true;

  this->card_ = nullptr;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount(this->mount_point_.c_str(), &host, &slot_config, &mount_config, &this->card_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s (0x%x)", esp_err_to_name(ret), (unsigned)ret);
    this->mounted_ = false;
    this->mounting_ = false;
    return;
  }
  this->mounted_ = true;
  sdmmc_card_print_info(stdout, this->card_);
  ESP_LOGI(TAG, "SD card mounted");
  this->mounting_ = false;

  // Small grace period so VFS dirent/stat are ready before first fopen
  vTaskDelay(pdMS_TO_TICKS(50));
}

void SDCardMount::dump_config() {
  ESP_LOGCONFIG(TAG, "SD Card:");
  ESP_LOGCONFIG(TAG, "  Mount point: %s", this->mount_point_.c_str());
  ESP_LOGCONFIG(TAG, "  Pins: CLK=%d CMD=%d D0=%d", this->clk_pin_, this->cmd_pin_, this->d0_pin_);
  ESP_LOGCONFIG(TAG, "  Width: %d", this->width_);
  ESP_LOGCONFIG(TAG, "  Max files: %d", this->max_files_);
  ESP_LOGCONFIG(TAG, "  Allocation unit: %d KB", this->alloc_unit_kb_);
  ESP_LOGCONFIG(TAG, "  Status: %s", this->mounted_ ? "mounted" : "not mounted");
}

void SDCardMount::power_on_() {
  if (this->power_pin_ < 0)
    return;
  gpio_config_t cfg{};
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pin_bit_mask = 1ULL << this->power_pin_;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&cfg);
  int on_level = this->power_active_low_ ? 0 : 1;
  gpio_set_level((gpio_num_t) this->power_pin_, on_level);
  if (this->power_on_delay_ms_ > 0) {
    vTaskDelay(pdMS_TO_TICKS(this->power_on_delay_ms_));
  }
}


void SDCardMount::unmount_() {
  if (this->mounted_) {
    esp_vfs_fat_sdcard_unmount(this->mount_point_.c_str(), this->card_);
    this->mounted_ = false;
    this->card_ = nullptr;
    ESP_LOGI(TAG, "SD card unmounted");
  }
}

void SDCardMount::unmount() { this->unmount_(); }

}  // namespace sd_card_cc
}  // namespace esphome
