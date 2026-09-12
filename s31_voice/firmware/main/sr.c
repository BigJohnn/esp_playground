/* 板上的两级离线识别：AFE -> WakeNet 唤醒词 -> MultiNet 中文命令词。
 *
 * 为什么是两级而不是一直开着命令词识别：MultiNet 一直跑既费电又容易误触发
 * （电视里说一句"关灯"灯就灭了）。WakeNet 模型小、常驻代价低，用它当门禁，
 * 门开了才让 MultiNet 上场。
 *
 * 两条任务的分工是 esp-sr 的固有形状：feed 任务只管把 I2S 的数据搬进 AFE，
 * fetch 任务拿降噪+VAD 之后的干净数据去做识别。中间隔着 AFE 的环形缓冲，
 * 识别慢一点也不会把麦克风的数据丢掉。
 *
 * 唤醒之后这里还顺手把这句话录下来（M4）。MultiNet 认出来就把录音丢掉，
 * 快路径一分钱额外开销都不花；认不出来才把这段 PCM 交出去让服务端做开放式识别。
 * 关键在于录音必须在"知道认不认得出"之前就开始 —— 等 MultiNet 超时再录，
 * 那句话早说完了。
 */
#include "sr.h"

#include "board.h"

#include <math.h>
#include <string.h>

#include "esp_afe_sr_models.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "model_path.h"

static const char *TAG = "sr";

#define MAX_COMMANDS 32
#define TEXT_MAX     48

typedef struct {
    char text[TEXT_MAX];
    char phonemes[64];
    bool used;
} command_slot_t;

static command_slot_t s_commands[MAX_COMMANDS];   /* 正在用的那张，只由 detect 任务写 */
static command_slot_t s_staged[MAX_COMMANDS];    /* 暂存，网络任务往这儿写 */
static int s_command_count;
static volatile bool s_started;         /* detect 任务已经在跑了 */
static volatile bool s_reload_pending;  /* 暂存表里有新东西，等空档换上去 */

static esp_codec_dev_handle_t s_codec;
static srmodel_list_t *s_models;
static const esp_afe_sr_iface_t *s_afe;
static esp_afe_sr_data_t *s_afe_data;
static const esp_mn_iface_t *s_mn;
static model_iface_data_t *s_mn_data;

static sr_event_cb_t s_cb;
static void *s_ctx;

/* 兜底录音。6 秒足够放下任何一句控灯的话（16k/16bit 单声道 = 192KB），
 * 放 PSRAM，内部 RAM 一个字节都不占。 */
#define REC_MAX_SAMPLES  (6 * 16000)
static int16_t *s_rec;
static volatile bool s_rec_busy;      /* 上一段还在被消费者用着，别覆盖它 */
static volatile bool s_muted;         /* 放 TTS 期间喂静音，见 sr_set_muted */
/* 连续对话：主任务处理完一条命令后置上它，detect_task 下一帧就直接开窗，
 * 不等唤醒词。用 volatile int 而不是加锁 —— 写的一边只写、读的一边读完就清零，
 * 这个竞态最坏的结果是窗口晚开一帧（32ms），不值得为它引入一把锁。 */
static volatile int s_relisten_ms;

/* VAD 判静音之后再确认多久算一句话说完。
 * 只要 240ms 就够，因为 vadnet1_medium 自己已经有 992ms 的迟滞
 * （日志里的 min noise:992 ms）—— 它说"静音"的时候，实际上已经安静一秒了。
 * 再叠一个长窗口纯粹是往兜底路径上白加延迟。 */
#define SILENCE_MS_TO_END  400

/* 掐头的时候往前多留这么多。实测 vadnet 报"开口"比真正的第一个音晚 ~300ms，
 * 留 300 正好卡在边界上（服务端存下来的音频头部余量是 0.00s）。
 * 多留的这点音频只值 8KB 上传，切掉第一个字却是致命的。 */
#define PREROLL_MS  500

void sr_commands_begin(void)
{
    memset(s_staged, 0, sizeof(s_staged));
}

esp_err_t sr_add_command(int id, const char *text, const char *phonemes)
{
    if (id < 0 || id >= MAX_COMMANDS || !text || !phonemes) {
        return ESP_ERR_INVALID_ARG;
    }
    strlcpy(s_staged[id].text, text, TEXT_MAX);
    strlcpy(s_staged[id].phonemes, phonemes, sizeof(s_staged[id].phonemes));
    s_staged[id].used = true;
    return ESP_OK;
}

/* 暂存表和在用的表内容一样吗。一样就不用重建词图 —— 重建要几百毫秒，
 * 而轮询大多数时候拿到的都是同一张表。 */
static bool staged_differs(void)
{
    return memcmp(s_staged, s_commands, sizeof(s_staged)) != 0;
}

static void apply_staged(void)
{
    memcpy(s_commands, s_staged, sizeof(s_commands));
    s_command_count = 0;
    for (int i = 0; i < MAX_COMMANDS; i++) {
        if (s_commands[i].used) {
            s_command_count++;
        }
    }
}

void sr_commands_commit(void)
{
    if (!s_started) {
        apply_staged();      /* 还没起来，直接换，sr_start 里再注册进模型 */
        return;
    }
    if (staged_differs()) {
        s_reload_pending = true;   /* 让 detect 任务在空档里换 */
    }
}

int sr_command_count(void)
{
    return s_command_count;
}

static esp_err_t register_commands(bool first_time)
{
    if (first_time) {
        esp_mn_commands_alloc(s_mn, s_mn_data);
    } else {
        /* 重建而不是增删：服务端给的是一张完整的表，逐条 diff 出增删改
         * 只会多一份会跟真相跑偏的状态。清空重来几百毫秒，值。 */
        esp_mn_commands_clear();
    }
    for (int i = 0; i < MAX_COMMANDS; i++) {
        if (s_commands[i].used) {
            esp_mn_commands_add(i, s_commands[i].phonemes);
        }
    }
    esp_mn_error_t *err = esp_mn_commands_update();
    if (err && err->num > 0) {
        /* 一条词进不去模型，通常是拼音拼错了或者音节太少。
         * 报出来而不是静默跳过 —— 否则现场只会表现为"这句话它就是不认" */
        for (int i = 0; i < err->num; i++) {
            ESP_LOGE(TAG, "命令词进不了模型: %s", err->phrases[i]->string);
        }
    }
    esp_mn_active_commands_print();
    return ESP_OK;
}

static void probe_feed(const int16_t *interleaved, int frames);   /* 定义在下面的探针一节 */

/* 只管搬运：I2S -> AFE。这条任务不能被任何慢操作卡住，
 * 一旦它停下来，麦克风的数据就真的丢了（I2S DMA 会覆盖）。 */
static void feed_task(void *arg)
{
    int chunk = s_afe->get_feed_chunksize(s_afe_data);
    int channels = s_afe->get_feed_channel_num(s_afe_data);
    /* I2S 收双声道（左=麦克风，右=DAC 回环），AFE 只要麦克风那一路，
     * 所以中间隔一次拆交织。多出来的一路不喂给 AFE：AFE 是按 "M"（单麦、无参考）
     * 建的，喂两路进去它会当成麦克风阵列。 */
    size_t i2s_bytes = (size_t)chunk * BOARD_I2S_CHANNELS * sizeof(int16_t);
    size_t afe_bytes = (size_t)chunk * channels * sizeof(int16_t);
    int16_t *i2s_buf = heap_caps_malloc(i2s_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    int16_t *buf = heap_caps_malloc(afe_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!i2s_buf || !buf) {
        ESP_LOGE(TAG, "feed 缓冲分配失败");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "feed: I2S 每帧 %d 采样 x %d 声道 -> AFE %d 声道 (%.0f ms)",
             chunk, BOARD_I2S_CHANNELS, channels, chunk * 1000.0f / 16000.0f);

    /* 每两秒报一次电平。没有这个，"没唤醒"有两种完全不同的可能 ——
     * 麦克风根本没收到声音，还是收到了但模型不认 —— 从日志上分不出来。 */
    int64_t next_report = 0;
    int64_t peak_hold = 0;
    int frames = 0;
    int64_t sum_sq = 0;

    while (1) {
        if (esp_codec_dev_read(s_codec, i2s_buf, i2s_bytes) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        for (int i = 0; i < chunk; i++) {
            buf[i] = i2s_buf[i * BOARD_I2S_CHANNELS + BOARD_CH_MIC];
        }
        probe_feed(i2s_buf, chunk);
        if (s_muted) {
            /* 照读不误、只是把内容清零：不读的话 I2S RX 的 DMA 会溢出，
             * 解除静音那一刻涌进来的是一堆积压的旧数据。
             * 喂零而不是干脆不喂，是为了让 AFE 的时间轴保持连续。 */
            memset(buf, 0, afe_bytes);
        }
        s_afe->feed(s_afe_data, buf);

        for (int i = 0; i < chunk * channels; i++) {   /* buf 是拆完交织的麦克风那路 */
            int v = buf[i] < 0 ? -buf[i] : buf[i];
            sum_sq += (int64_t)buf[i] * buf[i];
            if (v > peak_hold) {
                peak_hold = v;
            }
        }
        frames++;
        int64_t now = esp_timer_get_time();
        if (now >= next_report) {
            if (next_report) {
                int64_t n = (int64_t)frames * chunk * channels;
                int rms = n ? (int)sqrt((double)sum_sq / (double)n) : 0;
                ESP_LOGI(TAG, "麦克风电平 RMS=%d 峰值=%d%s", rms, (int)peak_hold,
                         peak_hold > 2000 ? "  <- 有说话" : "");
            }
            next_report = now + 2000000;
            peak_hold = 0;
            sum_sq = 0;
            frames = 0;
        }
    }
}


/* ---- DAC 回环探针 ----
 *
 * ES8311 把 DAC 数据镜像到 I2S 输入的右声道（REG44=0x58）。喇叭还没接上的时候，
 * 这是唯一能**实测**"板子到底有没有把音频送出去"的地方 —— 在此之前只有间接推断：
 * write() 返回 OK、耗时等于音频时长、寄存器读回来是对的。那些都是"没有反证"，
 * 不是"有正证"。这里做的是正证：放一个 1kHz 正弦，看右声道里有没有 1kHz。
 *
 * 注意它只验到数字段（I2S TX -> DAC 数字通路 -> 音量/静音）。
 * 模拟输出、NS4150B、喇叭仍然验不了，那要真喇叭。
 *
 * 用 Goertzel 而不是 FFT：只关心一个频点，Goertzel 每样本三次乘加就够，
 * 而 FFT 要在这条不能被卡住的搬运任务里分内存做蝶形运算。 */
static volatile bool s_probe_on;
static double s_probe_coeff;
static double s_probe_q1, s_probe_q2;
static double s_probe_ref_sq, s_probe_mic_sq;
static int s_probe_n;

static void probe_feed(const int16_t *interleaved, int frames)
{
    if (!s_probe_on) {
        return;
    }
    for (int i = 0; i < frames; i++) {
        double ref = interleaved[i * BOARD_I2S_CHANNELS + BOARD_CH_DAC_REF];
        double mic = interleaved[i * BOARD_I2S_CHANNELS + BOARD_CH_MIC];
        double q0 = s_probe_coeff * s_probe_q1 - s_probe_q2 + ref;
        s_probe_q2 = s_probe_q1;
        s_probe_q1 = q0;
        s_probe_ref_sq += ref * ref;
        s_probe_mic_sq += mic * mic;
        s_probe_n++;
    }
}

void sr_ref_probe_start(int freq_hz)
{
    s_probe_q1 = s_probe_q2 = 0;
    s_probe_ref_sq = s_probe_mic_sq = 0;
    s_probe_n = 0;
    s_probe_coeff = 2.0 * cos(2.0 * M_PI * freq_hz / 16000.0);
    s_probe_on = true;
}

void sr_ref_probe_stop(sr_ref_probe_t *out)
{
    s_probe_on = false;
    if (!out) {
        return;
    }
    int n = s_probe_n ? s_probe_n : 1;
    out->samples = s_probe_n;
    out->ref_rms = (float)sqrt(s_probe_ref_sq / n);
    out->mic_rms = (float)sqrt(s_probe_mic_sq / n);
    double mag2 = s_probe_q1 * s_probe_q1 + s_probe_q2 * s_probe_q2
                  - s_probe_coeff * s_probe_q1 * s_probe_q2;
    out->tone_amp = (float)(2.0 * sqrt(mag2 < 0 ? 0 : mag2) / n);
}

static void emit(sr_event_t ev, int id, const char *text, float prob,
                 const int16_t *pcm, size_t samples)
{
    if (!s_cb) {
        return;
    }
    sr_result_t r = {
        .event = ev, .command_id = id, .text = text, .prob = prob,
        .pcm = pcm, .samples = samples,
    };
    s_cb(&r, s_ctx);
}

static void detect_task(void *arg)
{
    int mn_chunk = s_mn->get_samp_chunksize(s_mn_data);
    int afe_chunk = s_afe->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG, "detect: AFE 每帧 %d 采样, MultiNet 要 %d 采样", afe_chunk, mn_chunk);
    if (afe_chunk != mn_chunk) {
        /* 两边帧长不一致就没法直接把 AFE 的输出喂给 MultiNet。
         * 真碰上了要加一层重分帧，先明确报出来而不是喂错长度的数据。 */
        ESP_LOGE(TAG, "帧长不一致，命令词识别会不准");
    }
    /* 一帧 32ms。用实际帧长换算，别把 32 写死在代码里。 */
    const int frame_ms = afe_chunk * 1000 / 16000;
    const int silence_frames_to_end = frame_ms > 0 ? SILENCE_MS_TO_END / frame_ms : 19;

    bool listening = false;
    int64_t listen_until = 0;
    size_t rec_len = 0;         /* 采样数 */
    bool recording = false;     /* 这一轮是否真的在往 s_rec 里写 */
    /* 端点检测的状态机。关键在于不能直接拿 vad_state==SPEECH 当"用户开口了"：
     * 唤醒词自己的尾巴就在 AFE 的输出里，而 vadnet 有近 1 秒的迟滞，
     * 所以刚唤醒完 VAD 一定还是 SPEECH。第一版就栽在这儿 —— 计数从唤醒词
     * 的尾音开始跑，1.7 秒后"判定说完了"，而用户那时才刚开口。
     * 所以要等它先落到 SILENCE（唤醒词的尾巴过去了），再把 SPEECH 当成开口。 */
    bool armed = false;         /* 唤醒词的尾音已经过去 */
    bool spoke = false;         /* armed 之后真的检测到过说话 */
    size_t speech_start = 0;    /* 开口那一刻在录音里的位置（采样） */
    int silence_frames = 0;
    vad_state_t last_vad = VAD_SILENCE;

    while (1) {
        afe_fetch_result_t *res = s_afe->fetch(s_afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            continue;
        }

        if (!listening) {
            /* 连续对话：上一条命令执行成功之后，不要求唤醒词直接再开一次窗。
             * 放在 wakenet 判断之前 —— 此刻 WakeNet 刚被重新打开，
             * 而我们要的正是"跳过它"。 */
            int again = s_relisten_ms;
            if (again > 0) {
                s_relisten_ms = 0;
                listening = true;
                listen_until = esp_timer_get_time() + (int64_t)again * 1000;
                rec_len = 0;
                /* armed=true：这一轮前面没有唤醒词，也就没有尾音要等过去。
                 * 首轮那套"先等 VAD 落到 SILENCE"的逻辑在这儿反而会多等近一秒。 */
                armed = true;
                spoke = false;
                speech_start = 0;
                silence_frames = 0;
                last_vad = VAD_SILENCE;
                /* 追问窗口**也录**兜底音频。最初的设计是不录 —— 理由是全量 ASR
                 * 在有噪音时基本废掉。但那条理由已经不成立了：唤醒和每一轮追问
                 * 现在都会让服务端把音乐压下去（见 net_notify_wake），背景是安静的。
                 *
                 * 而不录的代价很实在：板上词表只有那几条，用户在追问窗口里说
                 * 「打开收音机」「下一首」这类不在表里的话，会被**静默丢弃** ——
                 * 没有回话、没有灯、什么都没有，看起来就是坏了。 */
                recording = (s_rec != NULL) && !s_rec_busy;
                s_afe->disable_wakenet(s_afe_data);
                s_mn->clean(s_mn_data);
                emit(SR_EVENT_WAKE, -1, NULL, 0.0f, NULL, 0);
                continue;
            }
            /* 换词表只在这儿做：此刻没在识别命令词，MultiNet 的状态机是干净的。
             * 在唤醒之后换会把正在进行的一次识别打断，而且 esp_mn_commands_update()
             * 要重建词图，那期间的 detect() 结果不可信。 */
            if (s_reload_pending) {
                s_reload_pending = false;
                apply_staged();
                register_commands(false);
                ESP_LOGI(TAG, "命令词表已热更新，现在 %d 条", s_command_count);
            }
            if (res->wakeup_state != WAKENET_DETECTED) {
                continue;
            }
            ESP_LOGI(TAG, "听到唤醒词（第 %d 个词）", res->wake_word_index);
            listening = true;
            listen_until = esp_timer_get_time() +
                           (int64_t)CONFIG_S31_COMMAND_TIMEOUT_MS * 1000;
            rec_len = 0;
            armed = false;
            spoke = false;
            speech_start = 0;
            silence_frames = 0;
            last_vad = VAD_SPEECH;   /* 唤醒词刚说完，此刻必然还在 SPEECH */
            /* 上一段兜底录音还没被消费者还回来，这一轮就只走命令词，不录。 */
            recording = (s_rec != NULL) && !s_rec_busy;
            /* 唤醒后关掉 WakeNet：说命令词的这几秒里再触发一次唤醒毫无意义，
             * 还会把 MultiNet 的状态机打乱。 */
            s_afe->disable_wakenet(s_afe_data);
            s_mn->clean(s_mn_data);
            emit(SR_EVENT_WAKE, -1, NULL, 0.0f, NULL, 0);
            continue;
        }

        /* 先录再识别：这段 PCM 只有 MultiNet 认不出来的时候才用得上，
         * 但那时候已经来不及录了。 */
        if (recording) {
            size_t n = (size_t)res->data_size / sizeof(int16_t);
            if (rec_len + n > REC_MAX_SAMPLES) {
                n = REC_MAX_SAMPLES - rec_len;
            }
            if (n) {
                memcpy(s_rec + rec_len, res->data, n * sizeof(int16_t));
                rec_len += n;
            }
        }

        vad_state_t vad = res->vad_state;
        if (vad != last_vad) {
            ESP_LOGD(TAG, "VAD %s @%.2fs (%.1f dBFS)",
                     vad == VAD_SPEECH ? "-> 说话" : "-> 静音",
                     rec_len / 16000.0f, res->data_volume);
            last_vad = vad;
        }
        if (!armed) {
            if (vad == VAD_SILENCE) {
                armed = true;          /* 唤醒词的尾音过去了，从现在起才算数 */
            }
        } else if (vad == VAD_SPEECH) {
            if (!spoke) {
                speech_start = rec_len;
            }
            spoke = true;
            silence_frames = 0;
        } else if (spoke) {
            silence_frames++;
        }

        esp_mn_state_t st = s_mn->detect(s_mn_data, res->data);
        bool spoke_and_stopped = spoke && silence_frames >= silence_frames_to_end;
        bool out_of_time = (st == ESP_MN_STATE_TIMEOUT) ||
                           (esp_timer_get_time() > listen_until);

        if (st == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *r = s_mn->get_results(s_mn_data);
            int id = r->num > 0 ? r->command_id[0] : -1;
            float prob = r->num > 0 ? r->prob[0] : 0.0f;
            const char *text = (id >= 0 && id < MAX_COMMANDS && s_commands[id].used)
                                   ? s_commands[id].text : NULL;
            ESP_LOGI(TAG, "命令词 id=%d prob=%.2f -> %s", id, prob, text ? text : "(未知)");
            emit(SR_EVENT_COMMAND, id, text, prob, NULL, 0);   /* 录音直接丢掉 */
        } else if (spoke_and_stopped || out_of_time) {
            /* 超时的时候 armed 还是 false，说明 VAD 从唤醒到现在一次都没落到静音
             * —— 那就是有人一直在说话，这段录音是有内容的，别扔。 */
            bool worth_sending = spoke || !armed;
            if (!worth_sending) {
                ESP_LOGI(TAG, "唤醒后没人说话，回到待唤醒");
                emit(SR_EVENT_TIMEOUT, -1, NULL, 0.0f, NULL, 0);
            } else if (recording && rec_len > 0) {
                /* 掐掉开口之前的那段静音 —— 唤醒词和命令词之间人总要停一下，
                 * 那一秒既白占上传带宽，也让 STT 多一段无用的输入。
                 * 往前留 PREROLL_MS 余量：VAD 判"开口"晚于真正的第一个音。 */
                const size_t preroll = 16000 / 1000 * PREROLL_MS;
                size_t from = (spoke && speech_start > preroll) ? speech_start - preroll : 0;
                ESP_LOGI(TAG, "不在命令词表里，录了 %.2fs（掐掉开头 %.2fs）交给服务端（%s）",
                         (rec_len - from) / 16000.0f, from / 16000.0f,
                         spoke_and_stopped ? "VAD 判定说完" : "等到超时");
                /* 所有权交出去，消费者用完调 sr_release_utterance() 还回来。 */
                s_rec_busy = true;
                emit(SR_EVENT_UTTERANCE, -1, NULL, 0.0f, s_rec + from, rec_len - from);
            } else {
                ESP_LOGW(TAG, "说了话但没录上（上一段还在处理），只能放掉");
                emit(SR_EVENT_TIMEOUT, -1, NULL, 0.0f, NULL, 0);
            }
        } else {
            continue;   /* 还在听 */
        }

        listening = false;
        recording = false;
        s_mn->clean(s_mn_data);
        s_afe->enable_wakenet(s_afe_data);
    }
}

void sr_release_utterance(void)
{
    s_rec_busy = false;
}

void sr_set_muted(bool muted)
{
    s_muted = muted;
}

void sr_listen_again(int ms)
{
    s_relisten_ms = ms > 0 ? ms : 0;
}

esp_err_t sr_start(esp_codec_dev_handle_t codec, sr_event_cb_t cb, void *ctx)
{
    s_codec = codec;
    s_cb = cb;
    s_ctx = ctx;

    s_models = esp_srmodel_init("model");
    if (!s_models || s_models->num == 0) {
        ESP_LOGE(TAG, "model 分区里没有模型。sdkconfig 里选了唤醒词/命令词吗？"
                      "烧录时 srmodels.bin 有没有一起烧进去？");
        return ESP_ERR_NOT_FOUND;
    }
    for (int i = 0; i < s_models->num; i++) {
        ESP_LOGI(TAG, "模型 %d: %s", i, s_models->model_name[i]);
    }

    /* "M" = 一路麦克风、没有回放参考通道。
     * 板上只有一颗模拟麦，既没有麦克风阵列可做波束形成，也没把喇叭信号接回来，
     * 所以 AEC/BSS 用不上 —— afe_config_check() 会据此自动关掉它们。
     * 代价是放 TTS 的时候会听见自己（M4 要处理），M3 不放音，先不管。 */
    afe_config_t *cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!cfg) {
        return ESP_FAIL;
    }
    cfg->aec_init = false;
    cfg->se_init = false;
    cfg->vad_init = true;
    cfg->wakenet_init = true;
    cfg->agc_init = true;                       /* 单麦距离一远电平就掉，交给 AGC 拉回来 */
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;   /* 板上 16MB PSRAM，别去挤内部 RAM */
    afe_config_check(cfg);

    s_afe = esp_afe_handle_from_config(cfg);
    s_afe_data = s_afe->create_from_config(cfg);
    afe_config_free(cfg);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "AFE 创建失败");
        return ESP_FAIL;
    }
    s_afe->print_pipeline(s_afe_data);

    char *mn_name = esp_srmodel_filter(s_models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (!mn_name) {
        ESP_LOGE(TAG, "没有中文命令词模型（esp32s31 上要选 mn7_cn）");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "命令词模型: %s", mn_name);
    s_mn = esp_mn_handle_from_name(mn_name);
    s_mn_data = s_mn->create(mn_name, CONFIG_S31_COMMAND_TIMEOUT_MS);
    if (!s_mn_data) {
        ESP_LOGE(TAG, "MultiNet 创建失败");
        return ESP_FAIL;
    }
    register_commands(true);

    /* 录音缓冲放 PSRAM。分配不到就退化成"只有命令词"，不影响 M3 的功能。 */
    s_rec = heap_caps_malloc(REC_MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!s_rec) {
        ESP_LOGW(TAG, "兜底录音缓冲分配失败，只能识别命令词表里的话");
    }

    /* feed 优先级要高于 detect：搬不动数据就是真丢音频，识别慢一帧只是延迟。
     * 两条任务分到不同核上，AFE 的重活和识别的重活才不会互相抢。 */
    s_started = true;
    xTaskCreatePinnedToCore(feed_task, "sr_feed", 4 * 1024, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(detect_task, "sr_detect", 8 * 1024, NULL, 5, NULL, 1);
    return ESP_OK;
}
