/* 回话播放：POST /tts -> 边收边写 ES8311。
 *
 * 不先把整段音频收进内存再播：一句"亮度调到百分之八十"合成出来接近 100KB，
 * 收完再播白等半秒，而这半秒正好加在兜底路径本来就不短的延迟上。
 * esp_http_client 的手动读模式让第一包到手就能出声。
 */
#include "voice.h"

#include <string.h>

#include "audio_hw.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "net.h"
#include "sr.h"

static const char *TAG = "voice";

static esp_codec_dev_handle_t s_codec;

void voice_init(esp_codec_dev_handle_t codec)
{
    s_codec = codec;
}

#if CONFIG_S31_HAS_SPEAKER
static esp_err_t write_sink(const void *pcm, size_t bytes, void *ctx)
{
    int *total = (int *)ctx;
    *total += (int)bytes;
    /* 服务端给的是单声道，I2S 走的是双声道，展开这一步交给 audio_hw。
     * 它内部是阻塞写：DMA 排满了就在那儿等，
     * 天然把下载速度压到播放速度，不需要额外的流控。 */
    return audio_hw_write_mono(s_codec, pcm, bytes);
}
#endif

esp_err_t voice_say(const char *text)
{
    if (!s_codec || !text || !text[0]) {
        return ESP_ERR_INVALID_ARG;
    }
#if !CONFIG_S31_HAS_SPEAKER
    /* 没接喇叭就别去取音频了。合成 + 下载 + 实时播放合计一两秒，
     * 而这段时间里板子是静音的，既听不见回话也听不见新的唤醒词 ——
     * 纯亏。回话文本 do_utterance 已经打过日志，这里只留个记号。 */
    ESP_LOGI(TAG, "（无喇叭）本该念：%s", text);
    return ESP_OK;
#else
    int64_t t0 = esp_timer_get_time();
    int bytes = 0;

    sr_set_muted(true);
    esp_err_t err = net_fetch_tts(text, write_sink, &bytes);

    /* 补一段静音再解除静音。最后一次 write 只是把数据交给了 DMA，
     * 这时候立刻开麦，喇叭里还在响的那几十毫秒会被自己录进去。
     * 顺带也把 DMA 里的尾巴推干净，否则最后半个字听不见。 */
    static const int16_t silence[1600] = { 0 };   /* 单声道 100ms @16k */
    audio_hw_write_mono(s_codec, silence, sizeof(silence));
    audio_hw_write_mono(s_codec, silence, sizeof(silence));
    sr_set_muted(false);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "回话「%s」%.2fs 音频，用了 %d ms", text,
                 bytes / 2 / 16000.0f, (int)((esp_timer_get_time() - t0) / 1000));
    }
    return err;
#endif
}
