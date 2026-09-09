#include "audio_hw.h"

#include "board.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_hw";

static i2c_master_bus_handle_t s_i2c_bus;
static i2s_chan_handle_t s_tx_chan;
static i2s_chan_handle_t s_rx_chan;

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_i2c_bus);
}

esp_err_t audio_hw_i2c_scan(void)
{
    if (s_i2c_bus == NULL) {
        ESP_RETURN_ON_ERROR(i2c_init(), TAG, "i2c init failed");
    }
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "I2C 发现设备: 0x%02X%s", addr,
                     (addr == 0x18 || addr == 0x19) ? "  <- 应该就是 ES8311" : "");
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "I2C 上一个设备都没有。检查 SDA=%d SCL=%d 是否正确",
                 BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

/* ES8311 是全双工单芯片，收发共用同一个 I2S 端口（同一套 BCLK/LRCK/MCLK）。 */
static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;   /* 播完自动填 0，避免尾音重复出现爆音 */
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan), TAG, "new i2s chan");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BOARD_SAMPLE_RATE),
        /* 立体声。ES8311 只有一路 ADC 和一路 DAC，多出来的那个槽不是浪费：
         * 输入的右槽是 DAC 回环参考（REG44=0x58），输出两个槽写同样的数据。
         * TX/RX 共用 BCLK/LRCK，所以两边必须同为立体声，不能一边单一边双。 */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK_GPIO,
            .bclk = BOARD_I2S_BCLK_GPIO,
            .ws   = BOARD_I2S_WS_GPIO,
            .dout = BOARD_I2S_DOUT_GPIO,
            .din  = BOARD_I2S_DIN_GPIO,
            .invert_flags = { false, false, false },
        },
    };
    /* ES8311 需要 MCLK = 256 * fs */
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &std_cfg), TAG, "init tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &std_cfg), TAG, "init rx");
    return ESP_OK;
}

esp_err_t audio_hw_init(esp_codec_dev_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(out_dev != NULL, ESP_ERR_INVALID_ARG, TAG, "out_dev is NULL");

    if (s_i2c_bus == NULL) {
        ESP_RETURN_ON_ERROR(i2c_init(), TAG, "i2c init failed");
    }
    ESP_RETURN_ON_ERROR(i2s_init(), TAG, "i2s init failed");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = s_rx_chan,
        .tx_handle = s_tx_chan,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "new i2s data if");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_0,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl_if, ESP_FAIL, TAG, "new i2c ctrl if");

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = BOARD_PA_CTRL_GPIO,
        .pa_reverted = false,          /* NS4150B CTRL 高有效 */
        .use_mclk = true,
        .digital_mic = false,          /* J6 是模拟驻极体麦 */
        .hw_gain = { .pa_voltage = 5.0f, .codec_dac_voltage = 3.3f },
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);
    ESP_RETURN_ON_FALSE(codec_if, ESP_FAIL, TAG, "es8311_codec_new failed");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    esp_codec_dev_handle_t dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(dev, ESP_FAIL, TAG, "esp_codec_dev_new failed");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = BOARD_BITS_PER_SAMPLE,
        .channel = BOARD_I2S_CHANNELS,
        .sample_rate = BOARD_SAMPLE_RATE,
    };
    ESP_RETURN_ON_ERROR(esp_codec_dev_open(dev, &fs), TAG, "codec open failed");

    ESP_LOGI(TAG, "ES8311 就绪: %d Hz / %d bit / %d ch（左=麦克风 右=DAC 回环）",
             BOARD_SAMPLE_RATE, BOARD_BITS_PER_SAMPLE, BOARD_I2S_CHANNELS);
    *out_dev = dev;
    return ESP_OK;
}

esp_err_t audio_hw_set_volume(esp_codec_dev_handle_t dev, int vol_pct)
{
    return esp_codec_dev_set_out_vol(dev, vol_pct);
}

esp_err_t audio_hw_set_mic_gain(esp_codec_dev_handle_t dev, float gain_db)
{
    return esp_codec_dev_set_in_gain(dev, gain_db);
}


esp_err_t audio_hw_play_tone(esp_codec_dev_handle_t dev, int freq_hz, int ms, int amplitude)
{
    const int frames = BOARD_SAMPLE_RATE / 1000 * ms;
    int16_t *tone = malloc((size_t)frames * BOARD_I2S_CHANNELS * sizeof(int16_t));
    if (!tone) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < frames; i++) {
        int16_t v = (int16_t)(amplitude * sin(2.0 * M_PI * freq_hz * i / BOARD_SAMPLE_RATE));
        for (int c = 0; c < BOARD_I2S_CHANNELS; c++) {
            tone[i * BOARD_I2S_CHANNELS + c] = v;
        }
    }
    esp_err_t err = esp_codec_dev_write(dev, tone,
                                        (size_t)frames * BOARD_I2S_CHANNELS * sizeof(int16_t));
    free(tone);
    return err;
}

esp_err_t audio_hw_write_mono(esp_codec_dev_handle_t dev, const void *pcm, size_t bytes)
{
    /* 分块展开而不是一次性分配：调用方送进来的可能是几十 KB 的 TTS 音频，
     * 而这条路要边收边播，本来就是一小块一小块来的。 */
    enum { CHUNK_FRAMES = 512 };
    /* static 而不是栈上：2KB 放在 action 任务那 6KB 的栈里太挤。
     * 代价是不可重入 —— 播放只有 voice_say 一个调用方，且它本身是串行的。 */
    static int16_t stereo[CHUNK_FRAMES * BOARD_I2S_CHANNELS];
    const int16_t *src = (const int16_t *)pcm;
    size_t frames_left = bytes / sizeof(int16_t);
    while (frames_left > 0) {
        size_t n = frames_left > CHUNK_FRAMES ? CHUNK_FRAMES : frames_left;
        for (size_t i = 0; i < n; i++) {
            for (int c = 0; c < BOARD_I2S_CHANNELS; c++) {
                stereo[i * BOARD_I2S_CHANNELS + c] = src[i];
            }
        }
        esp_err_t err = esp_codec_dev_write(dev, stereo,
                                            n * BOARD_I2S_CHANNELS * sizeof(int16_t));
        if (err != ESP_OK) {
            return err;
        }
        src += n;
        frames_left -= n;
    }
    return ESP_OK;
}

static void play_tone(esp_codec_dev_handle_t dev, const char *label)
{
    ESP_LOGW(TAG, ">>> %s：放 1 秒 1kHz 正弦", label);
    audio_hw_play_tone(dev, 1000, 1000, 12000);
    ESP_LOGW(TAG, "<<< %s：放完", label);
}

esp_err_t audio_hw_diag(esp_codec_dev_handle_t dev)
{
    ESP_LOGW(TAG, "---- 音频输出诊断 ----");
    /* 注意：codec 驱动把 PA 脚配成 GPIO_MODE_OUTPUT，那样输入缓冲是关的，
     * gpio_get_level() 一律读回 0 —— 会骗人。改成 INPUT_OUTPUT 才读得到真实电平，
     * 输出锁存器不受影响。 */
    gpio_set_direction(BOARD_PA_CTRL_GPIO, GPIO_MODE_INPUT_OUTPUT);
    ESP_LOGW(TAG, "PA_CTRL(GPIO%d) 真实电平 = %d  (NS4150B 高有效)",
             BOARD_PA_CTRL_GPIO, gpio_get_level(BOARD_PA_CTRL_GPIO));

    esp_codec_dev_set_out_vol(dev, 100);
    play_tone(dev, "A 原样");

    gpio_set_level(BOARD_PA_CTRL_GPIO, 1);
    ESP_LOGW(TAG, "强制把 PA 拉高，现在电平 = %d", gpio_get_level(BOARD_PA_CTRL_GPIO));
    vTaskDelay(pdMS_TO_TICKS(1500));
    play_tone(dev, "B 强制开 PA");

    ESP_LOGW(TAG, "---- 诊断结束 ----");
    return ESP_OK;
}
