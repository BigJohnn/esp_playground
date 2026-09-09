/* ESP32-S31-Function-CoreBoard-1 引脚定义
 *
 * 来源：官方原理图 esp32-s31-function-coreboard-1-schematics.pdf (V1.0, 2026-05-13)
 * 第 2 页模块 U1 右侧 IO50~IO57 上的网络标号，与第 3 页 ES8311/NS4150B 对应。
 * GPIO50~57 没有引到 J2 排针，是音频专用。
 */
#pragma once

#define BOARD_I2C_SCL_GPIO      50   /* ESP_I2C_SCL  -> ES8311 CCLK */
#define BOARD_I2C_SDA_GPIO      51   /* ESP_I2C_SDA  -> ES8311 CDATA */

#define BOARD_I2S_MCLK_GPIO     52   /* I2S_MCLK     -> ES8311 MCLK */
#define BOARD_I2S_BCLK_GPIO     53   /* I2S_SCLK     -> ES8311 SCLK */
#define BOARD_I2S_WS_GPIO       55   /* I2S_LRCK     -> ES8311 LRCK */
#define BOARD_I2S_DIN_GPIO      54   /* I2S_ASDOUT   <- ES8311 ADC（麦克风进来） */
#define BOARD_I2S_DOUT_GPIO     56   /* I2S_DSDIN    -> ES8311 DAC（送去喇叭） */

#define BOARD_PA_CTRL_GPIO      57   /* PA_CTRL      -> NS4150B CTRL，高有效 */
#define BOARD_RGB_LED_GPIO      60   /* RGB_CTRL     -> WS2812 DIN */
#define BOARD_BOOT_BTN_GPIO     61   /* BOOT 键，按下为低 */

/* 麦克风是 J6 驻极体模拟麦，直接进 ES8311 的 MIC1P/MIC1N —— 只有一路真声音。
 *
 * 但 I2S 按**双声道**收：ES8311 驱动会写 REG44=0x58（"internal reference ADCL+DACR"），
 * 把 DAC 的数据镜像到 I2S 输入的右声道 —— 本来是给 AEC 当参考通道用的。
 * 按单声道收就只拿到左边那路，白白丢掉这个"板子实际播出去了什么"的观测点。
 * 收成双声道之后：左 = 麦克风，右 = DAC 回环。见 sr.c 的 feed_task。
 *
 * 代价：I2S 收发带宽都翻倍（16k 立体声 = 64KB/s），播放时要把单声道复制成两声道。
 * 这点开销换来的是播放链路数字段可观测，以及以后上 AEC 时参考通道现成。 */
#define BOARD_MIC_CHANNELS      1
#define BOARD_I2S_CHANNELS      2
/* 左右声道在交织缓冲里的下标 */
#define BOARD_CH_MIC            0
#define BOARD_CH_DAC_REF        1
#define BOARD_SAMPLE_RATE       16000
#define BOARD_BITS_PER_SAMPLE   16
