#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/display/display_buffer.h"

#include <vector>

extern "C" {
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
}

namespace esphome {
namespace box3 {

static const char *const TAG_BOX3_DISPLAY = "box3_display";

// ESP32-S3-BOX-3 LCD pinout (ILI9342C)
static const int LCD_HOST = SPI2_HOST;
static const int LCD_WIDTH = 320;
static const int LCD_HEIGHT = 240;
static const int LCD_PIN_MOSI = 6;   // GPIO6  - LCD_SDA/MOSI
static const int LCD_PIN_SCLK = 7;   // GPIO7  - LCD_SCK
static const int LCD_PIN_CS = 5;     // GPIO5  - LCD_CS
static const int LCD_PIN_DC = 4;     // GPIO4  - LCD_DC
static const int LCD_PIN_RST = 48;   // GPIO48 - LCD_RST
static const int LCD_PIXEL_CLOCK_HZ = 40 * 1000 * 1000;  // 40 MHz

class Box3Display : public display::DisplayBuffer, public Component {
 public:
  void setup() override {
    ESP_LOGCONFIG(TAG_BOX3_DISPLAY, "Initializing ESP32-S3-BOX-3 LCD (custom driver)...");
    this->init_lcd_();
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG_BOX3_DISPLAY, "ESP32-S3-BOX-3 custom display");
    LOG_DISPLAY("  ", "Box3Display", this);
  }

  display::DisplayType get_display_type() override {
    return display::DisplayType::DISPLAY_TYPE_COLOR;
  }

  int get_width_internal() override { return LCD_WIDTH; }
  int get_height_internal() override { return LCD_HEIGHT; }

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) {
      return;
    }

    // Convert ESPHome Color (8/8/8) -> RGB565
    const uint8_t r = color.red;
    const uint8_t g = color.green;
    const uint8_t b = color.blue;
    const uint16_t rgb565 =
        static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));

    buffer_[y * LCD_WIDTH + x] = rgb565;
  }

  void update() override {
    this->do_update_();
  }

  void do_update_() {
    if (panel_ == nullptr || buffer_.empty()) {
      return;
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(
        panel_, 0, 0, LCD_WIDTH, LCD_HEIGHT, buffer_.data());
    if (err != ESP_OK) {
      ESP_LOGE(TAG_BOX3_DISPLAY, "esp_lcd_panel_draw_bitmap failed: %d", err);
    }
  }

  void init_lcd_() {
    // Allocate full-frame RGB565 buffer
    buffer_.assign(static_cast<size_t>(LCD_WIDTH) * LCD_HEIGHT, 0x0000);

    // SPI bus configuration
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LCD_PIN_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = LCD_PIN_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2;

    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(LCD_HOST),
                                       &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      // ESP_ERR_INVALID_STATE means bus was already initialized, which is fine.
      ESP_LOGE(TAG_BOX3_DISPLAY, "spi_bus_initialize failed: %d", err);
      return;
    }

    // SPI -> LCD panel I/O
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = LCD_PIN_DC;
    io_config.cs_gpio_num = LCD_PIN_CS;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = nullptr;
    io_config.user_ctx = nullptr;
    io_config.flags.dc_low_on_data = 0;
    io_config.flags.swap_color_bytes = 1;  // RGB565 byte order

    err = esp_lcd_new_panel_io_spi(
        static_cast<esp_lcd_spi_bus_handle_t>(LCD_HOST),
        &io_config, &io_handle_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG_BOX3_DISPLAY, "esp_lcd_new_panel_io_spi failed: %d", err);
      return;
    }

    // ILI9342C is compatible with ILI9341 driver in esp_lcd
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = LCD_PIN_RST;
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
    panel_config.bits_per_pixel = 16;

    err = esp_lcd_new_panel_ili9341(io_handle_, &panel_config, &panel_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG_BOX3_DISPLAY, "esp_lcd_new_panel_ili9341 failed: %d", err);
      return;
    }

    // Basic panel init sequence
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));

    // Orientation: try to match ESP32-S3-BOX-3 default (320x240 landscape)
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, true));

    // Turn on display
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));
  }

  esp_lcd_panel_io_handle_t io_handle_{nullptr};
  esp_lcd_panel_handle_t panel_{nullptr};
  std::vector<uint16_t> buffer_;
};

}  // namespace box3
}  // namespace esphome

