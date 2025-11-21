#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/display/display_buffer.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstdio>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"

namespace esphome {
namespace box3_lcd {

static const char *const TAG_BOX3_LCD = "box3_lcd";

// ESP32-S3-BOX-3 LCD configuration (ILI9342C)
// Match BSP: uses SPI3_HOST for LCD
static const int LCD_HOST = SPI3_HOST;
static const int LCD_WIDTH = 320;
static const int LCD_HEIGHT = 240;
static const int LCD_PIN_MOSI = 6;   // GPIO6  - LCD_SDA/MOSI
static const int LCD_PIN_SCLK = 7;   // GPIO7  - LCD_SCK
static const int LCD_PIN_CS = 5;     // GPIO5  - LCD_CS
static const int LCD_PIN_DC = 4;     // GPIO4  - LCD_DC
static const int LCD_PIN_RST = 48;   // GPIO48 - LCD_RST
static const int LCD_PIXEL_CLOCK_HZ = 40 * 1000 * 1000;  // 40 MHz

// Simple 5x7 ASCII font for digits and some punctuation (row-based)
struct Glyph5x7 {
  char ch;
  uint8_t rows[7];  // each row: 5 LSBs used, bit 4 = leftmost pixel
};

static const Glyph5x7 FONT_5X7[] = {
    // Digits
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}},
    {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x1F}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}},
    {'3', {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}},
    {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}},
    {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}},
    {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    // Punctuation
    {':', {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04}},
    {'-', {0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00}},
    {',', {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'%', {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03}},
    // Uppercase letters (for weather text, etc.)
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}},
    {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
    {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}},
    {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'J', {0x1F, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'N', {0x11, 0x19, 0x15, 0x15, 0x13, 0x11, 0x11}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}},
    {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
    {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}},
    {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
    {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
};
static const int FONT_CHAR_WIDTH = 6;  // 5 pixels + 1 pixel spacing
static const int FONT_CHAR_HEIGHT = 7;

class Box3LCD : public display::DisplayBuffer {
 public:
  void setup() override {
    ESP_LOGCONFIG(TAG_BOX3_LCD, "Initializing ESP32-S3-BOX-3 LCD (box3_lcd)...");
    this->init_lcd_();
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG_BOX3_LCD, "ESP32-S3-BOX-3 Box3LCD");
    ESP_LOGCONFIG(TAG_BOX3_LCD, "  Resolution: %dx%d", LCD_WIDTH, LCD_HEIGHT);
  }

  display::DisplayType get_display_type() override {
    return display::DisplayType::DISPLAY_TYPE_COLOR;
  }

  int get_width_internal() override { return LCD_WIDTH; }
  int get_height_internal() override { return LCD_HEIGHT; }

  // Setters for data sources (wired from YAML via display.py)
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_ = s; }
  void set_humidity_sensor(sensor::Sensor *s) { this->humidity_ = s; }
  void set_radar_sensor(binary_sensor::BinarySensor *s) { this->radar_ = s; }
  void set_stock_sensor(sensor::Sensor *s) { this->stock_ = s; }
  void set_calendar_sensor(text_sensor::TextSensor *s) { this->calendar_ = s; }
  void set_news_sensor(text_sensor::TextSensor *s) { this->news_ = s; }
  void set_outdoor_temperature_sensor(sensor::Sensor *s) { this->outdoor_temperature_ = s; }
  void set_podcast_sensor(text_sensor::TextSensor *s) { this->podcast_ = s; }
  void set_time(time::RealTimeClock *t) { this->time_ = t; }

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) {
      return;
    }
    // Not used in our C++-only path
    (void) color;
  }

  void update() override {
    if (spi_dev_ == nullptr || buffer_.empty()) {
      return;
    }

    // Simple C++ dashboard: background + header + footer + center card
    const uint16_t COLOR_BG    = 0x0000;  // background: black
    const uint16_t COLOR_HEAD  = 0xF800;  // header: green (per your calibration)
    const uint16_t COLOR_CARD  = 0x07E0;  // card: red
    const uint16_t COLOR_FOOT  = 0x001F;  // footer: blue

    // Clear background
    std::fill(buffer_.begin(), buffer_.end(), COLOR_BG);

    // Header bar (top 40 px)
    const int header_h = 40;
    for (int y = 0; y < header_h; y++) {
      uint16_t *row = &buffer_[y * LCD_WIDTH];
      std::fill(row, row + LCD_WIDTH, COLOR_HEAD);
    }

    // Footer bar (bottom 40 px)
    const int footer_h = 40;
    for (int y = LCD_HEIGHT - footer_h; y < LCD_HEIGHT; y++) {
      uint16_t *row = &buffer_[y * LCD_WIDTH];
      std::fill(row, row + LCD_WIDTH, COLOR_FOOT);
    }

    // Center card rectangle
    const int card_w = 220;
    const int card_h = 120;
    const int card_x0 = (LCD_WIDTH  - card_w) / 2;
    const int card_y0 = (LCD_HEIGHT - card_h) / 2;
    const int card_x1 = card_x0 + card_w;
    const int card_y1 = card_y0 + card_h;

    for (int y = card_y0; y < card_y1; y++) {
      uint16_t *row = &buffer_[y * LCD_WIDTH];
      for (int x = card_x0; x < card_x1; x++) {
        row[x] = COLOR_CARD;
      }
    }

    // ---- Overlay data-driven elements ----
    // Temperature & humidity bars inside the card (left/right halves)
    const int inner_margin = 10;
    const int bar_area_h = card_h - 2 * inner_margin;
    const int bar_area_y1 = card_y1 - inner_margin;

    auto clamp01 = [](float v) -> float {
      if (v < 0.0f) return 0.0f;
      if (v > 1.0f) return 1.0f;
      return v;
    };

    // Temperature bar on left half of card
    if (this->temperature_ != nullptr && this->temperature_->has_state()) {
      float t = this->temperature_->state;  // °C
      // Map 0..40°C -> 0..1
      float norm = clamp01((t - 0.0f) / 40.0f);
      int bar_h = static_cast<int>(norm * bar_area_h);
      int bar_x0 = card_x0 + inner_margin;
      int bar_x1 = card_x0 + card_w / 2 - inner_margin;
      int bar_y0 = bar_area_y1 - bar_h;

      const uint16_t COLOR_TEMP_BAR = COLOR_HEAD;  // use green for temperature bar

      for (int y = bar_y0; y < bar_area_y1; y++) {
        uint16_t *row = &buffer_[y * LCD_WIDTH];
        for (int x = bar_x0; x < bar_x1; x++) {
          row[x] = COLOR_TEMP_BAR;
        }
      }
    }

    // Humidity bar on right half of card
    if (this->humidity_ != nullptr && this->humidity_->has_state()) {
      float h = this->humidity_->state;  // %
      // Map 20..35% -> 0..1 to make changes more visible around your current range (~23-25%)
      float norm = clamp01((h - 20.0f) / 15.0f);
      int bar_h = std::max(4, static_cast<int>(norm * bar_area_h));  // keep a minimum visible height
      int bar_x0 = card_x0 + card_w / 2 + inner_margin;
      int bar_x1 = card_x1 - inner_margin;
      int bar_y0 = bar_area_y1 - bar_h;

      const uint16_t COLOR_HUM_BAR = COLOR_FOOT;  // use blue for humidity bar

      for (int y = bar_y0; y < bar_area_y1; y++) {
        uint16_t *row = &buffer_[y * LCD_WIDTH];
        for (int x = bar_x0; x < bar_x1; x++) {
          row[x] = COLOR_HUM_BAR;
        }
      }
    }

    // Presence indicator on header (small square on right)
    if (this->radar_ != nullptr && this->radar_->has_state()) {
      bool present = this->radar_->state;
      const int ind_size = 18;
      const int ind_x1 = LCD_WIDTH - inner_margin;
      const int ind_x0 = ind_x1 - ind_size;
      const int ind_y0 = (header_h - ind_size) / 2;
      const int ind_y1 = ind_y0 + ind_size;

      uint16_t color = present ? COLOR_CARD : COLOR_BG;  // red if present, black if not
      for (int y = ind_y0; y < ind_y1; y++) {
        uint16_t *row = &buffer_[y * LCD_WIDTH];
        for (int x = ind_x0; x < ind_x1; x++) {
          row[x] = color;
        }
      }
    }

    // Simple stock bar in footer (left-to-right based on stock value)
    if (this->stock_ != nullptr && this->stock_->has_state()) {
      float s = this->stock_->state;
      // If stock is not a valid number (nan/inf), skip drawing the bar
      if (!std::isfinite(s)) {
        // leave footer as plain blue background
      } else {
        // Map 100..200 -> 0..1 (tweak this range to your stock price)
        float norm = clamp01((s - 100.0f) / 100.0f);
        int bar_w = std::max(4, static_cast<int>(norm * (LCD_WIDTH - 2 * inner_margin)));
        int bar_x0 = inner_margin;
        int bar_x1 = bar_x0 + bar_w;
        int bar_y0 = LCD_HEIGHT - footer_h + inner_margin;
        int bar_y1 = LCD_HEIGHT - inner_margin;

        const uint16_t COLOR_STOCK_BAR = COLOR_CARD;  // red bar for stock

        for (int y = bar_y0; y < bar_y1; y++) {
          uint16_t *row = &buffer_[y * LCD_WIDTH];
          for (int x = bar_x0; x < bar_x1; x++) {
            row[x] = COLOR_STOCK_BAR;
          }
        }
      }
    }

    // ---- Text overlays ----
    const uint16_t COLOR_TEXT = 0xFFFF;  // try white-ish for text

    // Header: show latest Home Assistant podcast time in a compact, local format
    // Use a 2x scaled font to make it larger and visually bolder.
    const int HEADER_TEXT_Y = 8;
    if (this->podcast_ != nullptr && this->podcast_->has_state()) {
      std::string raw = this->podcast_->state;
      std::string text = this->format_podcast_header_(raw);
      // Limit to one line
      const int HEADER_SCALE = 2;
      size_t max_chars = (LCD_WIDTH - 8) / (FONT_CHAR_WIDTH * HEADER_SCALE);
      if (text.size() > max_chars) {
        text = text.substr(0, max_chars);
      }
      this->draw_text_scaled_(4, HEADER_TEXT_Y, text, COLOR_TEXT, HEADER_SCALE);
    }

    // Center card: numeric temperature and humidity (scaled 2x for better readability)
    const int CENTER_SCALE = 2;
    if (this->temperature_ != nullptr && this->temperature_->has_state()) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%.1f", this->temperature_->state);
      this->draw_text_scaled_(card_x0 + inner_margin,
                              card_y0 + inner_margin,
                              buf, COLOR_TEXT, CENTER_SCALE);
    }
    if (this->humidity_ != nullptr && this->humidity_->has_state()) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%.0f%%", this->humidity_->state);
      this->draw_text_scaled_(card_x0 + card_w / 2 + inner_margin,
                              card_y0 + inner_margin,
                              buf, COLOR_TEXT, CENTER_SCALE);
    }

    // Footer: show outside temperature + forecast from Home Assistant (scaled 2x)
    const int FOOTER_TEXT_Y = LCD_HEIGHT - footer_h + 4;
    const int FOOTER_SCALE = 2;
    if (this->outdoor_temperature_ != nullptr && this->outdoor_temperature_->has_state() &&
        this->news_ != nullptr && this->news_->has_state()) {
      char temp_buf[8];
      std::snprintf(temp_buf, sizeof(temp_buf), "%.0fC", this->outdoor_temperature_->state);
      std::string weather = this->news_->state;
      std::string line = temp_buf;
      line += " ";
      line += weather;

      size_t max_chars = (LCD_WIDTH - 2 * inner_margin) / (FONT_CHAR_WIDTH * FOOTER_SCALE);
      if (line.size() > max_chars) {
        line = line.substr(0, max_chars);
      }
      this->draw_text_scaled_(inner_margin, FOOTER_TEXT_Y, line, COLOR_TEXT, FOOTER_SCALE);
    } else if (this->outdoor_temperature_ != nullptr && this->outdoor_temperature_->has_state()) {
      char temp_buf[8];
      std::snprintf(temp_buf, sizeof(temp_buf), "%.0fC", this->outdoor_temperature_->state);
      this->draw_text_scaled_(inner_margin, FOOTER_TEXT_Y, temp_buf, COLOR_TEXT, FOOTER_SCALE);
    } else if (this->news_ != nullptr && this->news_->has_state()) {
      std::string weather = this->news_->state;
      std::string line = weather;
      size_t max_chars = (LCD_WIDTH - 2 * inner_margin) / (FONT_CHAR_WIDTH * FOOTER_SCALE);
      if (line.size() > max_chars) {
        line = line.substr(0, max_chars);
      }
      this->draw_text_scaled_(inner_margin, FOOTER_TEXT_Y, line, COLOR_TEXT, FOOTER_SCALE);
    } else {
      this->draw_text_scaled_(inner_margin, FOOTER_TEXT_Y, "--", COLOR_TEXT, FOOTER_SCALE);
    }

    // Flush to panel
    this->flush_();
  }

  void init_lcd_() {
    // Allocate full-frame RGB565 buffer, default to black
    buffer_.assign(static_cast<size_t>(LCD_WIDTH) * LCD_HEIGHT, 0x0000);

    // Configure SPI bus
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
      // ESP_ERR_INVALID_STATE means bus was already initialized, which is acceptable.
      ESP_LOGE(TAG_BOX3_LCD, "spi_bus_initialize failed: %d", err);
      return;
    }

    // Configure GPIOs for DC and RST
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // SPI device configuration (CS handled by SPI driver)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = LCD_PIXEL_CLOCK_HZ;
    devcfg.mode = 0;
    devcfg.spics_io_num = LCD_PIN_CS;
    devcfg.queue_size = 1;
    devcfg.flags = 0;

    err = spi_bus_add_device(static_cast<spi_host_device_t>(LCD_HOST),
                             &devcfg, &spi_dev_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG_BOX3_LCD, "spi_bus_add_device failed: %d", err);
      return;
    }

    // Hardware reset (panel reset is active-high per BSP)
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_RST), 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Vendor-specific ILI9342C init sequence from ESP-BOX-3 BSP
    struct LcdInitCmd {
      uint8_t cmd;
      const uint8_t *data;
      uint8_t data_len;
      uint8_t delay_ms;
    };

    static const uint8_t c8_data[]  = {0xFF, 0x93, 0x42};
    static const uint8_t c0_data[]  = {0x0E, 0x0E};
    static const uint8_t c5_data[]  = {0xD0};
    static const uint8_t c1_data[]  = {0x02};
    static const uint8_t b4_data[]  = {0x02};
    static const uint8_t e0_data[]  = {0x00, 0x03, 0x08, 0x06, 0x13, 0x09,
                                       0x39, 0x39, 0x48, 0x02, 0x0A, 0x08,
                                       0x17, 0x17, 0x0F};
    static const uint8_t e1_data[]  = {0x00, 0x28, 0x29, 0x01, 0x0D, 0x03,
                                       0x3F, 0x33, 0x52, 0x04, 0x0F, 0x0E,
                                       0x37, 0x38, 0x0F};
    static const uint8_t b1_data[]  = {0x00, 0x1B};
    static const uint8_t mactl_data[] = {0x08};
    static const uint8_t pixfmt_data[] = {0x55};
    static const uint8_t b7_data[]  = {0x06};

    static const LcdInitCmd init_cmds[] = {
        {0xC8, c8_data,        sizeof(c8_data),        0},
        {0xC0, c0_data,        sizeof(c0_data),        0},
        {0xC5, c5_data,        sizeof(c5_data),        0},
        {0xC1, c1_data,        sizeof(c1_data),        0},
        {0xB4, b4_data,        sizeof(b4_data),        0},
        {0xE0, e0_data,        sizeof(e0_data),        0},
        {0xE1, e1_data,        sizeof(e1_data),        0},
        {0xB1, b1_data,        sizeof(b1_data),        0},
        {0x36, mactl_data,     sizeof(mactl_data),     0},
        {0x3A, pixfmt_data,    sizeof(pixfmt_data),    0},
        {0xB7, b7_data,        sizeof(b7_data),        0},
        // Sleep out & display on with delays
        {0x11, nullptr,        0,                      120},
        {0x29, nullptr,        0,                      20},
    };

    for (auto &ic : init_cmds) {
      this->write_cmd_(ic.cmd);
      if (ic.data != nullptr && ic.data_len > 0) {
        this->write_data_(ic.data, ic.data_len);
      }
      if (ic.delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ic.delay_ms));
      }
    }
  }

  void write_cmd_(uint8_t cmd) {
    if (spi_dev_ == nullptr) return;
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 0);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_transmit(spi_dev_, &t);
  }

  void write_data_(const uint8_t *data, size_t len) {
    if (spi_dev_ == nullptr || data == nullptr || len == 0) return;
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 1);

    const size_t max_chunk = 1024;
    while (len > 0) {
      size_t chunk = len > max_chunk ? max_chunk : len;
      spi_transaction_t t = {};
      t.length = chunk * 8;
      t.tx_buffer = data;
      spi_device_transmit(spi_dev_, &t);
      data += chunk;
      len -= chunk;
    }
  }

  void set_address_window_(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // Column address set
    this->write_cmd_(0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    this->write_data_(data, 4);

    // Page address set
    this->write_cmd_(0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    this->write_data_(data, 4);
  }

  void flush_() {
    // Set full frame window and write GRAM
    this->set_address_window_(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    this->write_cmd_(0x2C);  // Memory write

    // Send rows in reverse order (vertical flip) and mirror horizontally to match panel orientation
    uint8_t rowbuf[LCD_WIDTH * 2];  // one row of RGB565 (little-endian)
    for (int y = 0; y < LCD_HEIGHT; y++) {
      int src_y = LCD_HEIGHT - 1 - y;  // flip vertically
      const uint16_t *src = &buffer_[src_y * LCD_WIDTH];
      for (int x = 0; x < LCD_WIDTH; x++) {
        int src_x = LCD_WIDTH - 1 - x;  // flip horizontally
        uint16_t v = src[src_x];
        rowbuf[2 * x]     = static_cast<uint8_t>(v & 0xFF);
        rowbuf[2 * x + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
      }
      this->write_data_(rowbuf, LCD_WIDTH * 2);
    }
  }

  spi_device_handle_t spi_dev_{nullptr};
  std::vector<uint16_t> buffer_;

  // Data sources wired from YAML
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  binary_sensor::BinarySensor *radar_{nullptr};
  sensor::Sensor *stock_{nullptr};
  text_sensor::TextSensor *calendar_{nullptr};
  text_sensor::TextSensor *news_{nullptr};
  sensor::Sensor *outdoor_temperature_{nullptr};
  text_sensor::TextSensor *podcast_{nullptr};
  time::RealTimeClock *time_{nullptr};

  // --- Helper: set a single pixel in our RGB565 framebuffer ---
  inline void set_pixel_(int x, int y, uint16_t color) {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) {
      return;
    }
    buffer_[y * LCD_WIDTH + x] = color;
  }

  // --- Helper: find glyph for our 5x7 font ---
  const Glyph5x7 *find_glyph_(char c) {
    // Map lowercase to uppercase where possible
    if (c >= 'a' && c <= 'z') {
      c = static_cast<char>(c - 'a' + 'A');
    }
    for (const auto &g : FONT_5X7) {
      if (g.ch == c) return &g;
    }
    return nullptr;
  }

  // --- Helper: draw a single 5x7 character ---
  void draw_char_(int x, int y, char c, uint16_t color) {
    const Glyph5x7 *glyph = this->find_glyph_(c);
    if (glyph == nullptr) {
      // Unknown glyph: treat as space
      return;
    }
    for (int row = 0; row < FONT_CHAR_HEIGHT; row++) {
      uint8_t bits = glyph->rows[row];
      for (int col = 0; col < 5; col++) {
        // bit 4 is leftmost, bit 0 is rightmost
        if (bits & (1 << (4 - col))) {
          this->set_pixel_(x + col, y + row, color);
        }
      }
    }
  }

  // --- Helper: draw ASCII text using 5x7 font ---
  void draw_text_(int x, int y, const std::string &text, uint16_t color) {
    int cx = x;
    for (char c : text) {
      if (c == ' ') {
        cx += FONT_CHAR_WIDTH;
        continue;
      }
      this->draw_char_(cx, y, c, color);
      cx += FONT_CHAR_WIDTH;
      if (cx >= LCD_WIDTH) break;  // avoid overflow
    }
  }

  // --- Helper: draw a single 5x7 character, scaled up (used for bold/large header text) ---
  void draw_char_scaled_(int x, int y, char c, uint16_t color, int scale) {
    const Glyph5x7 *glyph = this->find_glyph_(c);
    if (glyph == nullptr || scale <= 0) {
      return;
    }
    for (int row = 0; row < FONT_CHAR_HEIGHT; row++) {
      uint8_t bits = glyph->rows[row];
      for (int col = 0; col < 5; col++) {
        // bit 4 is leftmost, bit 0 is rightmost
        if (bits & (1 << (4 - col))) {
          // simple nearest-neighbor scale: each pixel becomes a scale x scale block
          int px = x + col * scale;
          int py = y + row * scale;
          for (int dy = 0; dy < scale; dy++) {
            for (int dx = 0; dx < scale; dx++) {
              this->set_pixel_(px + dx, py + dy, color);
            }
          }
        }
      }
    }
  }

  // --- Helper: draw ASCII text using scaled 5x7 font ---
  void draw_text_scaled_(int x, int y, const std::string &text, uint16_t color, int scale) {
    if (scale <= 0) return;
    int cx = x;
    const int adv = FONT_CHAR_WIDTH * scale;
    for (char c : text) {
      if (c == ' ') {
        cx += adv;
        continue;
      }
      this->draw_char_scaled_(cx, y, c, color, scale);
      cx += adv;
      if (cx >= LCD_WIDTH) break;  // avoid overflow
    }
  }

  // --- Helper: format Home Assistant podcast event state into local, compact text ---
  std::string format_podcast_header_(const std::string &raw) {
    // Home Assistant event state is usually an ISO timestamp in UTC, e.g.
    // "2025-11-21T02:38:01.564+00:00"
    // For a compact display we show "MM-DD HH:MM" in local time (assume UTC+8).
    if (raw.size() < 16) {
      return raw;  // not a timestamp, just show as-is
    }

    size_t tpos = raw.find('T');
    if (tpos == std::string::npos || tpos < 10 || raw.size() < tpos + 5) {
      return raw;  // unexpected format, fallback
    }

    auto parse_two_digits = [&](size_t pos, int &out) -> bool {
      if (pos + 1 >= raw.size()) return false;
      char c1 = raw[pos];
      char c2 = raw[pos + 1];
      if (c1 < '0' || c1 > '9' || c2 < '0' || c2 > '9') return false;
      out = (c1 - '0') * 10 + (c2 - '0');
      return true;
    };

    int month = 0, day = 0, hour = 0, minute = 0;
    if (!parse_two_digits(5, month) ||
        !parse_two_digits(8, day) ||
        !parse_two_digits(tpos + 1, hour) ||
        !parse_two_digits(tpos + 4, minute)) {
      // Parsing failed, fall back to raw representation
      return raw;
    }

    // Convert UTC -> local time (assume UTC+8, e.g. Asia/Shanghai)
    int local_hour = hour + 8;
    if (local_hour >= 24) {
      local_hour -= 24;
      // For simplicity we don't roll the date; the visual difference is usually small.
    }

    char buf[20];
    std::snprintf(buf, sizeof(buf), "%02d-%02d %02d:%02d", month, day, local_hour, minute);
    return std::string(buf);
  }
};

}  // namespace box3_lcd
}  // namespace esphome
