#include "local_ir.h"
#include "sdkconfig.h"

#if CONFIG_S31_LOCAL_IR_ENABLED
#include <string.h>
#include "board.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "ir_nec.h"
#include "sr.h"

static const char *TAG = "local_ir";
static rmt_channel_handle_t s_tx;
static rmt_encoder_handle_t s_encoder;
static bool s_ready;
/* Single action worker owns these buffers. They outlive each RMT transaction. */
static ir_pulse_t s_pulses[IR_NEC_MAX_PULSES];
static rmt_symbol_word_t s_symbols[512];

static const struct {
    const char *text;
    const char *phonemes;
    uint16_t command;
} s_commands[] = {
    {"声音大一点", "sheng yin da yi dian", 0xFF00},
    {"声音小一点", "sheng yin xiao yi dian", 0xBA45},
};

bool local_ir_profile_enabled(void) { return true; }

void local_ir_stage_commands(void)
{
    for (unsigned i = 0; i < sizeof(s_commands) / sizeof(s_commands[0]); ++i) {
        sr_add_command(i, s_commands[i].text, s_commands[i].phonemes);
    }
}

esp_err_t local_ir_init(void)
{
    /* -1（未配置）必须在预处理期挡掉，不能交给 GPIO_IS_VALID_OUTPUT_GPIO：
     * 那个宏是 1ULL << pin，而 pin 是编译期常量，-1 会直接撞上
     * -Werror=shift-count-negative —— 也就是说按 Kconfig 的默认值开这个功能，
     * 代码根本编不过。这是真编了一次才发现的。 */
#if CONFIG_S31_IR_TX_GPIO < 0
    ESP_LOGE(TAG, "红外 TX 引脚未配置（-1），本地红外不启用");
    return ESP_ERR_INVALID_ARG;
#else
    const int pin = CONFIG_S31_IR_TX_GPIO;
    /* Audio, UART, LED and boot pins belong to the current CoreBoard firmware.
     * Other pins still require checking against the actual board schematic. */
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin) || pin >= BOARD_I2C_SCL_GPIO) {
        ESP_LOGE(TAG, "IR TX 引脚无效或与板载音频/调试引脚冲突: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }
    rmt_tx_channel_config_t cfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 512,
        .trans_queue_depth = 1,
        .flags.with_dma = true,
    };
    esp_err_t err = rmt_new_tx_channel(&cfg, &s_tx);
    if (err != ESP_OK) return err;
    rmt_copy_encoder_config_t encoder_cfg = {};   /* v6.1 里这个结构是空的，{0} 会被 -Werror 拒掉 */
    err = rmt_new_copy_encoder(&encoder_cfg, &s_encoder);
    if (err != ESP_OK) goto fail;
    rmt_carrier_config_t carrier = {
        .frequency_hz = 38000,
        .duty_cycle = 0.5f,
    };
    err = rmt_apply_carrier(s_tx, &carrier);
    if (err != ESP_OK) goto fail;
    err = rmt_enable(s_tx);
    if (err != ESP_OK) goto fail;
    s_ready = true;
    ESP_LOGI(TAG, "本地红外验证模式：TX GPIO%d，Tivoli 音量±；两条词仍需声学验收", pin);
    return ESP_OK;
fail:
    if (s_encoder) rmt_del_encoder(s_encoder);
    rmt_del_channel(s_tx);
    s_encoder = NULL;
    s_tx = NULL;
    return err;
#endif
}

/* RMT duration fields are 15 bits. A 96035us silence must be split without
 * inserting a mark or losing time; both halves can have the same level. */
static size_t pack_symbols(size_t pulse_count)
{
    memset(s_symbols, 0, sizeof(s_symbols));
    size_t half = 0;
    for (size_t i = 0; i < pulse_count; ++i) {
        uint32_t remaining = s_pulses[i].duration_us;
        while (remaining) {
            if (half / 2 >= sizeof(s_symbols) / sizeof(s_symbols[0])) return 0;
            uint32_t duration = remaining > 32767 ? 32767 : remaining;
            rmt_symbol_word_t *sym = &s_symbols[half / 2];
            if (half % 2) {
                sym->duration1 = duration;
                sym->level1 = s_pulses[i].mark;
            } else {
                sym->duration0 = duration;
                sym->level0 = s_pulses[i].mark;
            }
            remaining -= duration;
            ++half;
        }
    }
    return (half + 1) / 2;
}

bool local_ir_execute(const char *text, esp_err_t *result)
{
    if (!text || !result) return false;
    for (unsigned i = 0; i < sizeof(s_commands) / sizeof(s_commands[0]); ++i) {
        if (strcmp(text, s_commands[i].text) != 0) continue;
        *result = ESP_ERR_INVALID_STATE;
        if (!s_ready) return true;
        /* 一句话只按一下。重复帧数取 0 而不是 1：ESPHome 那条路
         * （remote_transmitter.transmit_nec, command_repeats: 1）发的就是
         * 「帧 + 结尾 mark」，不带 ditto，而 M6 的全部实测都是走它测出来的。
         * 真遥控器短按确实带一个 ditto，但那条波形在这台机器上没有被验证过 ——
         * 多一个 ditto 有可能被读成自动重复（一句话走两格音量），
         * 而红外没有回执，走错了也发现不了。取有硬件证据的那一个。 */
        size_t pulses = ir_nec_build(0x6B86, s_commands[i].command, 0,
                                     s_pulses, IR_NEC_MAX_PULSES);
        size_t symbols = pack_symbols(pulses);
        if (!pulses || !symbols) {
            *result = ESP_ERR_INVALID_SIZE;
            return true;
        }
        rmt_transmit_config_t tx_cfg = {0};
        *result = rmt_transmit(s_tx, s_encoder, s_symbols,
                               symbols * sizeof(s_symbols[0]), &tx_cfg);
        if (*result == ESP_OK) *result = rmt_tx_wait_all_done(s_tx, 1000);
        if (*result != ESP_OK) {
            /* Completion is uncertain. Stop output, refuse subsequent sends
             * until restart, and never route this command to the old IR board. */
            s_ready = false;
            rmt_disable(s_tx);
        }
        return true;
    }
    return false;
}
#else
esp_err_t local_ir_init(void) { return ESP_OK; }
bool local_ir_profile_enabled(void) { return false; }
void local_ir_stage_commands(void) {}
bool local_ir_execute(const char *text, esp_err_t *result) { return false; }
#endif
