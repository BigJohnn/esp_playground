/* ESP32-S31-Function-CoreBoard-1 —— 语音控灯
 *
 *   麦克风 -> ES8311 -> I2S -> AFE(降噪/VAD/AGC) -> WakeNet「你好小智」
 *          ├─ 认识 -> MultiNet 命令词 -> POST /command            （快，~0.5s）
 *          └─ 不认识 -> 整段 PCM -> POST /utterance -> 服务端 ASR  （慢，兜底）
 *                                 -> POST /tts -> 从喇叭念出回话（需 S31_HAS_SPEAKER）
 *
 * 两条路都只把"一句中文"交给服务端，意图解析和控灯留在服务端一处实现。
 *
 * 命令词表开机时从服务端拉（服务端是唯一来源，改词不用重烧固件）；
 * 拉不到就用编译进来的那份 —— 断网时唤醒和识别仍然能工作，只是控不了灯。
 *
 * RGB LED 就是状态指示：
 *   蓝  启动中 / 正在联网
 *   绿  待唤醒
 *   青  已唤醒，正在听命令
 *   白  正在执行命令词
 *   紫  这句话板上不认识，正在问服务端
 *   黄闪 听懂了，但这件事做不了
 *   红  出错
 */
#include <stdbool.h>
#include <string.h>

#include "audio_hw.h"
#include "board.h"
#include "esp_http_client.h"   /* ESP_ERR_HTTP_CONNECT，判"够不着"用 */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "net.h"
#include "sr.h"
#include "voice.h"

static const char *TAG = "main";

static led_strip_handle_t s_led;
static QueueHandle_t s_action_queue;

/* 交给 action_task 的一件活。两条路径共用一个队列，因为它们的后半段
 * （等服务端、点灯、恢复状态）完全一样，分成两条只会重复。 */
typedef struct {
    bool is_utterance;      /* true: pcm 是一整段没认出来的话；false: text 是命令词 */
    bool is_followup;       /* 这一轮是追问窗口（没说唤醒词） */
    char text[48];
    const int16_t *pcm;
    size_t samples;
} action_t;

/* 服务端拉不到时的兜底词表。和 server/intent.py 的 _MULTINET_COMMANDS 保持一致：
 * 那边是唯一来源，这里只是断网时的副本。
 * 每条都是 3-4 个音节 —— 两音节词实测漏识别率高得多，见 README 的 M3 实测数据。 */
static const struct {
    const char *text;
    const char *phonemes;
} k_default_commands[] = {
    { "打开台灯", "da kai tai deng" },
    { "关闭台灯", "guan bi tai deng" },
    { "亮一点",  "liang yi dian" },
    { "暗一点",  "an yi dian" },
    { "全亮模式", "quan liang mo shi" },
    { "黄光模式", "huang guang mo shi" },
    { "冷光模式", "leng guang mo shi" },
    { "阅读模式", "yue du mo shi" },
    { "夜灯模式", "ye deng mo shi" },
};

static void led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = BOARD_RGB_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
    };
    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led) != ESP_OK) {
        ESP_LOGW(TAG, "RGB LED 初始化失败（不致命，继续）");
        s_led = NULL;
    }
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led) {
        led_strip_set_pixel(s_led, 0, r, g, b);
        led_strip_refresh(s_led);
    }
}

/* 短促闪几下再回到底色。没接喇叭的时候这是唯一的"回话"通道，
 * 所以它不是装饰：兜底路径上"听懂了但做不了"跟"根本没听见"必须能分开。 */
static void led_blink(uint8_t r, uint8_t g, uint8_t b, int times)
{
    for (int i = 0; i < times; i++) {
        led_set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(140));
        led_set(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static char s_cmd_version[24];

static void on_server_command(int id, const char *text, const char *phonemes, void *ctx)
{
    sr_add_command(id, text, phonemes);
}

static void load_commands(void)
{
    sr_commands_begin();
    if (net_server_url() &&
        net_fetch_commands(on_server_command, NULL, s_cmd_version, sizeof(s_cmd_version)) == ESP_OK) {
        sr_commands_commit();
        return;
    }
    ESP_LOGW(TAG, "用编译进来的兜底词表");
    s_cmd_version[0] = '\0';
    for (int i = 0; i < (int)(sizeof(k_default_commands) / sizeof(k_default_commands[0])); i++) {
        sr_add_command(i, k_default_commands[i].text, k_default_commands[i].phonemes);
    }
    sr_commands_commit();
}

/* 词表轮询。问的是 /commands/version（几十字节），不是整张表 ——
 * 一分钟一次、常年不变的东西，不值得每次都搬 1.2KB 过来。
 * 版本变了才拉全表，然后 sr 在下一个"没在识别"的空档里换上去。 */
static void commands_task(void *arg)
{
    const int period_ms = CONFIG_S31_COMMANDS_POLL_S * 1000;
    /* 连续多少轮拿不到就重新找服务端。
     * 起因是一次真实故障：Mac 上和板子同网段的网卡掉了，发现服务回了一个
     * 连不上的地址，板子存下来之后**到重启为止都不会再改**，语音控灯彻底哑掉，
     * 而两边日志看着都正常。这条轮询本来就每分钟碰一次服务端，
     * 顺手让它兼当心跳：连不上就说明手里这个地址已经没用了，重新找一个。 */
    const int fails_before_rediscover = 3;
    int fails = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        if (!net_server_url()) {
            /* 开机时没找到（服务端还没起来 / Wi-Fi 还没好），就一直找下去。 */
            if (net_wifi_connected() && net_find_server(3000) == ESP_OK) {
                ESP_LOGI(TAG, "找到服务端了，补拉一次词表");
                load_commands();
                ESP_LOGI(TAG, "命令词 %d 条", sr_command_count());
            }
            continue;
        }
        char ver[24] = { 0 };
        if (net_fetch_commands_version(ver, sizeof(ver)) != ESP_OK) {
            if (++fails >= fails_before_rediscover) {
                ESP_LOGW(TAG, "连续 %d 次够不着 %s，重新找服务端",
                         fails, net_server_url());
                fails = 0;
                if (net_find_server(3000) == ESP_OK) {
                    load_commands();
                    ESP_LOGI(TAG, "命令词 %d 条", sr_command_count());
                }
            }
            continue;      /* 服务端一时不在，下一轮再说，别刷屏 */
        }
        fails = 0;
        if (strcmp(ver, s_cmd_version) == 0) {
            continue;
        }
        ESP_LOGI(TAG, "词表版本 %s -> %s，重新拉取",
                 s_cmd_version[0] ? s_cmd_version : "(无)", ver);
        sr_commands_begin();
        if (net_fetch_commands(on_server_command, NULL,
                               s_cmd_version, sizeof(s_cmd_version)) == ESP_OK) {
            sr_commands_commit();
        } else {
            ESP_LOGW(TAG, "拉词表失败，保持原表");
        }
    }
}

/* 识别回调跑在 sr 的 detect 任务上，那条任务不能被网络请求堵住
 * ——它一停，AFE 的环形缓冲就开始积压。所以这里只投递，活儿交给下面的任务。 */
static void on_sr_event(const sr_result_t *res, void *ctx)
{
    action_t act = { 0 };
    switch (res->event) {
    case SR_EVENT_WAKE:
        led_set(0, 24, 24);          /* 青：在听 */
        /* 立刻告诉服务端把音乐压低。放在这儿而不是等命令说完 ——
         * 唤醒词和命令词之间有 0.5~1 秒，正好够音量降下去，
         * 让**第一句命令**就落在安静背景上。
         * 实测不这么做的后果：放着歌说话，服务端收到的是「🎼我唱唱给的算」。 */
        net_notify_wake();
        break;
    case SR_EVENT_TIMEOUT:
        led_set(0, 24, 0);           /* 绿：回到待唤醒 */
        break;
    case SR_EVENT_COMMAND:
        if (res->text) {
            strlcpy(act.text, res->text, sizeof(act.text));
        }
        xQueueSend(s_action_queue, &act, 0);
        break;
    case SR_EVENT_UTTERANCE:
        act.is_utterance = true;
        act.is_followup = res->is_followup;
        act.pcm = res->pcm;
        act.samples = res->samples;
        /* 队列满就直接把录音还回去，否则 sr 那边会一直以为有人在用它。 */
        if (xQueueSend(s_action_queue, &act, 0) != pdTRUE) {
            ESP_LOGW(TAG, "动作队列满了，这段录音丢弃");
            sr_release_utterance();
        }
        break;
    }
}

/* 请求失败之后：如果失败的原因是"连不上"，那手里这个地址多半已经作废了
 * （服务端换了网卡/换了 IP，或者当初就拿到了一个没用的地址）。
 * 就地重新找一次 —— 用户刚说的这句已经废了，但下一句能用。
 * 只在连接层面的错误上做：HTTP 500 之类说明服务端在，重找没有意义。 */
static void recover_if_unreachable(esp_err_t err)
{
    if (err != ESP_ERR_HTTP_CONNECT && err != ESP_ERR_TIMEOUT) {
        return;
    }
    ESP_LOGW(TAG, "够不着服务端（%s），就地重新找一次", esp_err_to_name(err));
    if (net_find_server(2000) == ESP_OK) {
        load_commands();
        ESP_LOGI(TAG, "换到 %s，命令词 %d 条", net_server_url(), sr_command_count());
    }
}

/* 快路径：板上认出来的命令词，直接把中文交给服务端。
 * 不念回话 —— 灯自己亮/灭就是最快也最确定的反馈，再加一句 TTS 只会更慢更吵。 */
/* 连续对话：一条命令成功之后不用再说唤醒词。
 * 上限存在的理由很实际 —— 没有上限的话，一次误识别就可能让麦克风一直开着。
 * 5 轮足够覆盖"放歌 -> 下一首 -> 声音小点 -> 再下一首"这种真实串联。 */
#define FOLLOWUP_MS      3500
#define FOLLOWUP_MAX     5
static int s_followup_left;
/* 当前这个窗口是不是"我们问了问题在等回答"。由服务端在回复里带下来。
 * 只影响一件事：窗口里没听懂时该不该出声。见 net.h 里 net_followup_t 的说明。 */
static bool s_asking;

static void arm_followup(const net_followup_t *fu)
{
    if (!fu || !fu->wanted) {
        s_followup_left = 0;
        s_asking = false;
        return;
    }
    if (s_followup_left <= 0) {
        s_followup_left = FOLLOWUP_MAX;   /* 新的一串对话 */
    }
    if (--s_followup_left <= 0) {
        ESP_LOGI(TAG, "连续对话到上限了，下一句要重新唤醒");
        s_asking = false;
        return;
    }
    /* 窗口长度由服务端给，它才知道刚才说出去的是什么。
     * 3500ms 在「好的」后面正好，在"一，黑鸭子；二，龚玥；三，…要哪个？"
     * 后面完全不够 —— 用户还没张嘴窗口就关了。 */
    int ms = (fu->ms > 0) ? fu->ms : FOLLOWUP_MS;
    s_asking = fu->asking;
    ESP_LOGI(TAG, "继续听 %d ms（还剩 %d 轮%s）", ms, s_followup_left,
             s_asking ? "，在等回答" : "");
    led_set(0, 0, 24);                    /* 蓝：不用唤醒词，直接说 */
    sr_listen_again(ms);
}

static void do_command(const action_t *act)
{
    led_set(24, 24, 24);             /* 白：执行中 */
    char reply[128] = { 0 };
    bool ok = false;
    net_timing_t t = { 0 };
    int64_t t0 = esp_timer_get_time();
    net_followup_t fu = { 0 };
    esp_err_t err = net_send_command(act->text, reply, sizeof(reply), &ok, &fu, &t);
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    if (err == ESP_OK) {
        /* 这条路不念回话（灯自己亮/灭就是最快的反馈），所以开窗没有"等我说完"
         * 的问题，收到响应就能开。兜底那条路不一样，见 do_utterance。 */
        arm_followup(&fu);
        /* 拆成"服务端里"和"网络上"两段。快路径本来就只有几百毫秒，
         * 再快就得知道那几百毫秒到底是谁的 —— 灯泡自己（exec）还是 Wi-Fi。 */
        ESP_LOGI(TAG, "「%s」-> %s (%s, %d ms = 服务端 %d[控灯 %d] + 网络 %d)",
                 act->text, reply, ok ? "已执行" : "执行失败", ms,
                 t.server_ms, t.server_exec_ms, ms - t.server_ms);
    } else {
        ESP_LOGE(TAG, "「%s」发送失败: %s", act->text, esp_err_to_name(err));
        led_set(24, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(400));
        recover_if_unreachable(err);
    }
}

/* 兜底路径：板上不认识的一句话，整段音频交给服务端去听。
 * 这条路上要念回话 —— 用户说了句"没见过"的话，得知道它到底听成了什么、
 * 做了没有；灯没反应的时候尤其需要（"这个我还不会"）。 */
static void do_utterance(const action_t *act)
{
    led_set(24, 0, 24);              /* 紫：在问服务端 */
    char heard[96] = { 0 };
    char reply[128] = { 0 };
    bool ok = false;
    net_timing_t t = { 0 };
    int64_t t0 = esp_timer_get_time();
    net_followup_t fu = { 0 };
    esp_err_t err = net_send_utterance(act->pcm, act->samples * sizeof(int16_t),
                                       heard, sizeof(heard), reply, sizeof(reply), &ok,
                                       &fu, &t);
    /* 音频已经发完，缓冲马上还回去，下一句话才录得上。 */
    sr_release_utterance();

    if (err == ESP_OK && !ok && act->is_followup && !s_asking) {
        /* 追问窗口里没听懂，而且**我们并没有在问问题** —— 安静地算了。
         * 窗口是我们自己开的；屋里随便一点动静都会走到这儿，
         * 而对着动静说"这个我还不会"比什么都不说糟得多
         * （实测：每条成功的命令后面都跟一句，对着空气）。
         *
         * s_asking 时这个前提就不成立了：用户刚被我们问了"要哪个"，
         * 他答了一句，系统一声不吭 —— 他分不清是没听见还是答错了。
         * 所以那种情况要往下走，把回话念出来。 */
        ESP_LOGI(TAG, "追问窗口里没听懂（听成「%s」），不出声", heard);
        led_set(0, 24, 0);
        arm_followup(&fu);
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "兜底请求失败: %s", esp_err_to_name(err));
        led_set(24, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(400));
        recover_if_unreachable(err);
        return;
    }
    int total = (int)((esp_timer_get_time() - t0) / 1000);
    /* 一行把整条路的每一段都摆出来。之前只有一个总数，1.2s 和 3.9s 分不出病因；
     * 上传/STT/控灯/其它是四种完全不同的问题。 */
    ESP_LOGI(TAG, "服务端听成「%s」-> %s (%s)", heard, reply, ok ? "已执行" : "没执行");
    /* 服务端自报的 server 里含着"等 body"（recv），那一段和板子的 upload 是同一段时间，
     * 不扣掉的话"其它"会算成负数。 */
    int server_work = t.server_ms - t.server_recv_ms;
    ESP_LOGI(TAG, "  耗时 %d ms = 上传 %d(%.1fKB) + 服务端 %d[STT %d 控灯 %d] + 网络 %d",
             total, t.upload_ms, act->samples * 2 / 1024.0f,
             server_work, t.server_stt_ms, t.server_exec_ms,
             total - t.upload_ms - server_work);
    /* 听懂了但没做（"这个我还不会"、或者听成了一句没有意图的话）——
     * 灯不会变，用户光看着会以为板子没反应，所以这儿得自己出个声。 */
    if (!ok) {
        led_blink(24, 16, 0, 2);     /* 黄：收到了，但这件事做不了 */
    }
    voice_say(reply);
    /* 开窗放在 voice_say **之后**。它是阻塞的（写 DMA + 补一段静音 + 解除静音），
     * 所以这一行执行到的时刻，正好是喇叭刚停下来的那一刻 —— 窗口从用户
     * 能开口的时候才开始计时。放在前面的话，"一，黑鸭子；二，…要哪个？"
     * 这句话本身就要念五六秒，7 秒的窗口全花在听自己说话上。
     *
     * 这也是兜底路径以前根本没有连续对话的原因：这里压根没调过 arm_followup，
     * 于是"不用再说唤醒词"只对板上那十几条命令词成立，而点歌这类必须走
     * 服务端 ASR 的请求，窗口从来就没开过。 */
    arm_followup(&fu);
}

/* 没接喇叭时的开机自检：往 DAC 送一段 1kHz，看它有没有从 ES8311 的回环通道
 * 回到麦克风数据流的右声道。这是在没有喇叭的情况下，唯一能**正面证明**
 * "板子确实把音频送出去了"的办法 —— 在此之前只有"write 返回 OK、
 * 寄存器读回来是对的"这类反证。
 * 接了喇叭就不跑：那时候开机播报本身就是自检，而每次上电"嘀"一声很烦。 */
#if !CONFIG_S31_HAS_SPEAKER
static void loopback_check(esp_codec_dev_handle_t codec)
{
    const int amp = 12000;
    sr_ref_probe_start(1000);
    audio_hw_play_tone(codec, 1000, 300, amp);
    vTaskDelay(pdMS_TO_TICKS(150));     /* 等 feed 任务把最后几帧读完 */
    sr_ref_probe_t p = { 0 };
    sr_ref_probe_stop(&p);
    /* 判据用比例而不是绝对值：音量、DAC 增益都会缩放它，但"有没有这个频率"不会变。
     * 麦克风那一路同期贴着底噪，正好证明右声道不是串过来的。 */
    /* 幅度打不满是正常的：回环点在数字音量之后，音量设到 70 就该看到七成左右。
     * 真正的判据是"这个频率在不在"，以及麦克风那一路同期是不是安静的。 */
    ESP_LOGI(TAG, "播放自检：1kHz 幅度 %.0f（送出去 %d，%.0f%%，音量 70），"
                  "回环 RMS %.0f，同期麦克风 RMS %.0f，%d 采样",
             p.tone_amp, amp, 100.0f * p.tone_amp / amp, p.ref_rms, p.mic_rms, p.samples);
    if (p.tone_amp < amp * 0.1f) {
        ESP_LOGW(TAG, "播放自检没测到信号 —— I2S/DAC 这段有问题，不只是缺喇叭");
    }
}
#endif

static void action_task(void *arg)
{
    action_t act;
    while (1) {
        if (xQueueReceive(s_action_queue, &act, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!net_server_url()) {
            ESP_LOGW(TAG, "没连上服务端，这句话只能干听着");
            if (act.is_utterance) {
                sr_release_utterance();
            }
        } else if (act.is_utterance) {
            do_utterance(&act);
        } else if (act.text[0] == '\0') {
            ESP_LOGW(TAG, "识别到命令词但没有对应文本");
        } else {
            do_command(&act);
        }
        led_set(0, 24, 0);
    }
}

void app_main(void)
{
    led_init();
    led_set(0, 0, 24);
    ESP_LOGI(TAG, "=== S31 语音控灯 ===");

    if (audio_hw_i2c_scan() != ESP_OK) {
        led_set(24, 0, 0);
        return;
    }
    esp_codec_dev_handle_t codec = NULL;
    if (audio_hw_init(&codec) != ESP_OK) {
        ESP_LOGE(TAG, "音频初始化失败");
        led_set(24, 0, 0);
        return;
    }
    audio_hw_set_volume(codec, CONFIG_S31_SPEAKER_VOLUME);
    audio_hw_set_mic_gain(codec, (float)CONFIG_S31_MIC_GAIN_DB);
    /* audio_hw_diag(codec);  开机不跑，排查喇叭时手动打开 */

    /* 先联网再起识别：拿到命令词表之后再建 MultiNet，省掉一次重建词图。
     * 联不上也继续往下走 —— 唤醒和识别本来就不该依赖网络。 */
    if (net_wifi_start() == ESP_OK) {
        if (net_wifi_wait(15000)) {
            net_find_server(3000);
        } else {
            ESP_LOGW(TAG, "15 秒没连上 Wi-Fi，先离线跑着，后台继续重连");
        }
    }

    voice_init(codec);
    s_action_queue = xQueueCreate(4, sizeof(action_t));
    load_commands();
    ESP_LOGI(TAG, "命令词 %d 条", sr_command_count());

    if (sr_start(codec, on_sr_event, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "语音识别启动失败");
        led_set(24, 0, 0);
        return;
    }
    xTaskCreate(action_task, "action", 6 * 1024, NULL, 4, NULL);
    if (CONFIG_S31_COMMANDS_POLL_S > 0) {
        xTaskCreate(commands_task, "cmd_poll", 5 * 1024, NULL, 3, NULL);
    }

    led_set(0, 24, 0);
    ESP_LOGI(TAG, "就绪。说「你好小智」唤醒，然后说「打开台灯」。");
    /* "我起来了、而且连上服务端了" —— 这一句只有开机时说得着。
     * 接了喇叭就念出来（顺带每次上电自检一遍播放链路）；没接就闪两下绿灯，
     * 反正断网时底色也是绿的，光看常亮分不出连没连上。 */
    if (net_server_url()) {
        led_blink(0, 24, 0, 2);
        led_set(0, 24, 0);
        voice_say("语音助手就绪");
    }
#if !CONFIG_S31_HAS_SPEAKER
    loopback_check(codec);
#endif
}
