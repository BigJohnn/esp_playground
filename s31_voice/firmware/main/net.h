#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* Wi-Fi station。非阻塞启动，掉线自动重连。 */
esp_err_t net_wifi_start(void);
bool net_wifi_wait(int timeout_ms);
bool net_wifi_connected(void);

/* 找服务端。先 UDP 广播问一圈，问不到就退回 CONFIG_S31_SERVER_HOST。
 * 成功后地址记在模块内部，后面的 net_* 调用直接用。 */
esp_err_t net_find_server(int timeout_ms);
const char *net_server_url(void);

/* 一次请求的耗时拆解，单位毫秒。传 NULL 就是不关心。
 *
 * 为什么要拆：兜底路径实测 1.2~3.9s，只有一个端到端数字的时候，
 * 既可能是上传慢（Wi-Fi）、也可能是 STT 慢（模型）、还可能是灯泡慢（miIO/云）。
 * 三种病的药完全不同，不拆开就只能靠猜。
 *   upload  = 建连 + 把 body 写完（板子这边看得见的上行时间）
 *   wait    = 写完到响应头到手（服务端干活的时间，含网络回程）
 *   server_*= 服务端自报的分段，从响应 JSON 的 ms 字段里取
 *
 * 注意 server_recv_ms 和 upload_ms 量的是**同一段墙钟时间**：服务端的处理函数
 * 在 body 还没收完时就已经进去了，它等 body 的那段（recv）正是板子在上传的那段。
 * 所以"网络和协议开销" = 总时间 - upload - (server - server_recv)。
 * 第一版忘了扣，算出来是负数 —— 一个明显到不可能不发现的错，
 * 但如果它算出来是个看着合理的正数，就会被当成真数据用下去。 */
typedef struct {
    int upload_ms;
    int wait_ms;
    int server_ms;       /* 服务端从收到请求到回包 */
    int server_recv_ms;  /* 其中"等 body 收完"的部分，和 upload_ms 重叠 */
    int server_stt_ms;
    int server_exec_ms;  /* 真正去控灯花的时间 */
} net_timing_t;

/* GET /commands。回调对每条命令词调用一次；返回 ESP_OK 表示整张表都拿到了。
 * out_version 回填这张表的版本号（可传 NULL），配合 net_fetch_commands_version 用。 */
typedef void (*net_command_cb_t)(int id, const char *text, const char *phonemes, void *ctx);
esp_err_t net_fetch_commands(net_command_cb_t cb, void *ctx,
                             char *out_version, size_t version_len);

/* GET /commands/version。几十字节，给轮询用 —— 版本没变就别去拉那 1.2KB 的整表。 */
esp_err_t net_fetch_commands_version(char *out_version, size_t version_len);

/* POST /command，把识别到的中文句子交给服务端去解析意图并控灯。
 * reply 里回填服务端的回话文本（可传 NULL）。 */
esp_err_t net_send_command(const char *text, char *reply, size_t reply_len, bool *out_ok,
                           net_timing_t *timing);

/* ---- 兜底路径（M4）---- */

/* POST /utterance：把一整段 16k/16bit 单声道 PCM 交给服务端做开放式识别。
 * 服务端在这一次往返里就把灯控了，这里只回填识别文本和回话。 */
esp_err_t net_send_utterance(const void *pcm, size_t bytes,
                             char *text, size_t text_len,
                             char *reply, size_t reply_len, bool *out_ok,
                             net_timing_t *timing);

/* POST /tts：边收边回调，回调里直接往 codec 写。
 * 不先收完再播，首声延迟就只等第一个包，而不是整段音频。 */
typedef esp_err_t (*net_pcm_sink_t)(const void *pcm, size_t bytes, void *ctx);
esp_err_t net_fetch_tts(const char *text, net_pcm_sink_t sink, void *ctx);
