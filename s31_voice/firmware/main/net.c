/* Wi-Fi + 服务端发现 + HTTP 调用。
 *
 * 这里有意不放任何业务逻辑：板子只负责"识别出一句中文，交给服务端"，
 * 意图解析和控灯留在服务端一处实现，两条识别路径（板上 MultiNet / 服务端 ASR）
 * 才不会各自长出一套行为。
 */
#include "net.h"

#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "net";

/* 每个请求都带上共享令牌。放一个函数里，是为了不给"某条路径忘了加头"留缝 ——
 * 忘了加的那条就是唯一没上锁的控灯通道。 */
static void set_auth(esp_http_client_handle_t c)
{
    if (strlen(CONFIG_S31_API_TOKEN) == 0) {
        return;
    }
    char hdr[96];
    snprintf(hdr, sizeof(hdr), "Bearer %s", CONFIG_S31_API_TOKEN);
    esp_http_client_set_header(c, "Authorization", hdr);
}

/* 从响应 JSON 的 ms 字段里把服务端自报的分段耗时抄出来。 */
static void read_server_ms(cJSON *root, net_timing_t *t)
{
    if (!t) {
        return;
    }
    cJSON *ms = cJSON_GetObjectItem(root, "ms");
    if (!cJSON_IsObject(ms)) {
        return;
    }
    cJSON *v;
    if ((v = cJSON_GetObjectItem(ms, "server")) && cJSON_IsNumber(v)) t->server_ms = v->valueint;
    if ((v = cJSON_GetObjectItem(ms, "recv")) && cJSON_IsNumber(v)) t->server_recv_ms = v->valueint;
    if ((v = cJSON_GetObjectItem(ms, "stt")) && cJSON_IsNumber(v)) t->server_stt_ms = v->valueint;
    if ((v = cJSON_GetObjectItem(ms, "exec")) && cJSON_IsNumber(v)) t->server_exec_ms = v->valueint;
}

#define DISCOVERY_PORT 8791
#define DISCOVERY_PROBE "S31VOICE?"
#define DISCOVERY_REPLY "S31VOICE "

#define BIT_GOT_IP BIT0

static EventGroupHandle_t s_events;
static esp_netif_t *s_sta_netif;
static char s_server_url[64];
/* 重新发现可能被两条任务同时触发（轮询的心跳、和刚失败的那条命令）。
 * 让它们排队而不是同时广播 —— 两个都在改 s_server_url，撞上就是一个撕裂的字符串。 */
static SemaphoreHandle_t s_find_lock;

/* ---------------- Wi-Fi ---------------- */

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, BIT_GOT_IP);
        /* 不做指数退避：家里路由器重启的场景下，早连上比省那点功耗重要。 */
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "拿到 IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_events, BIT_GOT_IP);
    }
}

esp_err_t net_wifi_start(void)
{
    if (strlen(CONFIG_S31_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "没配 Wi-Fi SSID，只跑离线识别");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        on_wifi_event, NULL, NULL));

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, CONFIG_S31_WIFI_SSID, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, CONFIG_S31_WIFI_PASSWORD, sizeof(wc.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    /* 语音是持续上行的低延迟流量，DTIM 省电只会给每一跳加上百毫秒抖动
     * —— 这正是灯泡那 350ms 下限的成因，别在自己这端再叠一层。 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "连接 Wi-Fi \"%s\" …", CONFIG_S31_WIFI_SSID);
    return ESP_OK;
}

bool net_wifi_wait(int timeout_ms)
{
    if (!s_events) {
        return false;
    }
    return (xEventGroupWaitBits(s_events, BIT_GOT_IP, pdFALSE, pdTRUE,
                                pdMS_TO_TICKS(timeout_ms)) & BIT_GOT_IP) != 0;
}

bool net_wifi_connected(void)
{
    return s_events && (xEventGroupGetBits(s_events) & BIT_GOT_IP);
}

/* ---------------- 找服务端 ---------------- */

const char *net_server_url(void)
{
    return s_server_url[0] ? s_server_url : NULL;
}

/* 这个地址真的连得上吗。
 *
 * 加这一步的起因是一次真实故障：Mac 上和板子同网段的那张 USB 有线网卡掉了，
 * 代理软件的 TUN 网卡（198.18.0.1）顶上来接管了路由，发现服务于是"成功地"
 * 回了一个板子永远连不上的地址。板子把它存下来，之后每个请求都 CONNECT 失败，
 * 而日志两边看着都像正常。
 *
 * 教训不是"把那个地址加进黑名单"，是**别信别人给的地址，自己验一下**。
 * /health 不需要令牌，正是给这种探活用的。 */
static bool host_alive(const char *host, int port)
{
    char url[80];
    snprintf(url, sizeof(url), "http://%s:%d/health", host, port);
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        /* 短超时：这是在几个候选之间挑，不是在等一个慢服务。
         * 连不通的那个通常立刻 CONNECT 失败，根本用不到这个时限。 */
        .timeout_ms = 2000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return false;
    }
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    return err == ESP_OK && status == 200;
}

/* 从候选里挑第一个应答的，写进 s_server_url。 */
static bool adopt_first_alive(char hosts[][40], int n, int port)
{
    for (int i = 0; i < n; i++) {
        if (host_alive(hosts[i], port)) {
            snprintf(s_server_url, sizeof(s_server_url), "http://%s:%d", hosts[i], port);
            ESP_LOGI(TAG, "服务端在 %s（%d 个候选里的第 %d 个）", s_server_url, n, i + 1);
            return true;
        }
        ESP_LOGW(TAG, "候选 %s:%d 连不上，试下一个", hosts[i], port);
    }
    return false;
}

static esp_err_t find_server_locked(int timeout_ms);

esp_err_t net_find_server(int timeout_ms)
{
    if (!s_find_lock) {
        s_find_lock = xSemaphoreCreateMutex();
    }
    if (!s_find_lock ||
        xSemaphoreTake(s_find_lock, pdMS_TO_TICKS(timeout_ms + 5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t ret = find_server_locked(timeout_ms);
    xSemaphoreGive(s_find_lock);
    return ret;
}

static esp_err_t find_server_locked(int timeout_ms)
{
    s_server_url[0] = '\0';

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return ESP_FAIL;
    }
    int on = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    struct timeval tv = { .tv_sec = 0, .tv_usec = 300 * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* 两个目标地址都发：定向广播（192.168.0.255）和受限广播（255.255.255.255）。
     * 探灯泡时实测过受限广播可能完全没人应，反过来某些栈又只收受限广播 ——
     * 一个包 9 个字节，两个都发比赌哪个能通便宜得多。 */
    esp_netif_ip_info_t ip = { 0 };
    struct sockaddr_in targets[2] = { 0 };
    int n_targets = 0;
    if (s_sta_netif && esp_netif_get_ip_info(s_sta_netif, &ip) == ESP_OK && ip.ip.addr) {
        targets[n_targets].sin_family = AF_INET;
        targets[n_targets].sin_port = htons(DISCOVERY_PORT);
        targets[n_targets].sin_addr.s_addr = ip.ip.addr | ~ip.netmask.addr;
        n_targets++;
    }
    targets[n_targets].sin_family = AF_INET;
    targets[n_targets].sin_port = htons(DISCOVERY_PORT);
    targets[n_targets].sin_addr.s_addr = htonl(INADDR_BROADCAST);
    n_targets++;

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    char buf[192];

    while (esp_timer_get_time() < deadline) {
        for (int i = 0; i < n_targets; i++) {
            sendto(sock, DISCOVERY_PROBE, strlen(DISCOVERY_PROBE), 0,
                   (struct sockaddr *)&targets[i], sizeof(targets[i]));
        }
        int len = recv(sock, buf, sizeof(buf) - 1, 0);
        if (len <= (int)strlen(DISCOVERY_REPLY) ||
            strncmp(buf, DISCOVERY_REPLY, strlen(DISCOVERY_REPLY)) != 0) {
            continue;   /* 超时或者收到别的广播，再问一轮 */
        }
        buf[len] = '\0';

        cJSON *root = cJSON_Parse(buf + strlen(DISCOVERY_REPLY));
        if (!root) {
            continue;
        }
        cJSON *host = cJSON_GetObjectItem(root, "host");
        cJSON *port = cJSON_GetObjectItem(root, "port");
        cJSON *list = cJSON_GetObjectItem(root, "hosts");
        if (cJSON_IsNumber(port)) {
            /* 服务端给的是候选列表（老版本只有 host 一个，兼容着收）。
             * 一台机器可能同时挂在好几个网段上，只有它自己是猜不准的
             * —— 谁能连上只有板子这边试得出来。 */
            char hosts[4][40];
            int n = 0;
            cJSON *it = NULL;
            cJSON_ArrayForEach(it, list) {
                if (cJSON_IsString(it) && n < 4) {
                    strlcpy(hosts[n++], it->valuestring, sizeof(hosts[0]));
                }
            }
            if (n == 0 && cJSON_IsString(host)) {
                strlcpy(hosts[n++], host->valuestring, sizeof(hosts[0]));
            }
            if (n > 0 && adopt_first_alive(hosts, n, port->valueint)) {
                ret = ESP_OK;
            } else if (n > 0) {
                ESP_LOGW(TAG, "%d 个候选一个都连不上", n);
            }
        }
        cJSON_Delete(root);
        if (ret == ESP_OK) {
            break;
        }
    }
    close(sock);

    if (ret != ESP_OK && strlen(CONFIG_S31_SERVER_HOST) > 0) {
        /* 写死的那个也要探一下再认。不探的话，"没找到"会被伪装成"找到了"，
         * 后面每一条命令都失败，而日志停在这句乐观的话上。 */
        char fallback[1][40];
        strlcpy(fallback[0], CONFIG_S31_SERVER_HOST, sizeof(fallback[0]));
        ESP_LOGW(TAG, "广播没人应，试配置里的 %s", CONFIG_S31_SERVER_HOST);
        if (adopt_first_alive(fallback, 1, CONFIG_S31_SERVER_PORT)) {
            ret = ESP_OK;
        }
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "没找到服务端，识别结果只会打日志");
    }
    return ret;
}

/* ---------------- HTTP ---------------- */

/* 把整个响应体收进调用者给的缓冲区。响应都是几百字节的小 JSON，
 * 不值得为它上流式解析。 */
typedef struct {
    char *buf;
    size_t cap;
    size_t len;
} resp_t;

static esp_err_t on_http_event(esp_http_client_event_t *evt)
{
    resp_t *r = (resp_t *)evt->user_data;
    if (evt->event_id != HTTP_EVENT_ON_DATA || !r) {
        return ESP_OK;
    }
    size_t n = evt->data_len;
    if (r->len + n >= r->cap) {
        n = r->cap > r->len + 1 ? r->cap - r->len - 1 : 0;
    }
    if (n) {
        memcpy(r->buf + r->len, evt->data, n);
        r->len += n;
        r->buf[r->len] = '\0';
    }
    return ESP_OK;
}

static esp_err_t http_call(const char *path, esp_http_client_method_t method,
                           const char *body, char *out, size_t out_len)
{
    if (!s_server_url[0]) {
        return ESP_ERR_INVALID_STATE;
    }
    char url[128];
    snprintf(url, sizeof(url), "%s%s", s_server_url, path);

    resp_t resp = { .buf = out, .cap = out_len, .len = 0 };
    out[0] = '\0';

    esp_http_client_config_t cfg = {
        .url = url,
        .method = method,
        .timeout_ms = 5000,
        .event_handler = on_http_event,
        .user_data = &resp,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return ESP_FAIL;
    }
    set_auth(c);
    if (body) {
        esp_http_client_set_header(c, "Content-Type", "application/json");
        esp_http_client_set_post_field(c, body, strlen(body));
    }
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s 请求失败: %s", path, esp_err_to_name(err));
        return err;
    }
    if (status == 401) {
        ESP_LOGE(TAG, "%s 被拒：令牌不对。sdkconfig.secret 的 CONFIG_S31_API_TOKEN "
                      "要和 server/.env 的 API_TOKEN 完全一致", path);
        return ESP_ERR_INVALID_STATE;
    }
    if (status != 200) {
        ESP_LOGW(TAG, "%s 返回 HTTP %d", path, status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t net_fetch_commands_version(char *out_version, size_t version_len)
{
    char body[96];
    esp_err_t err = http_call("/commands/version", HTTP_METHOD_GET, NULL, body, sizeof(body));
    if (err != ESP_OK) {
        return err;
    }
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    cJSON *v = cJSON_GetObjectItem(root, "version");
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (cJSON_IsString(v) && out_version && version_len) {
        strlcpy(out_version, v->valuestring, version_len);
        ret = ESP_OK;
    }
    cJSON_Delete(root);
    return ret;
}

esp_err_t net_fetch_commands(net_command_cb_t cb, void *ctx,
                             char *out_version, size_t version_len)
{
    /* 12 条命令词的 JSON 大约 1.2KB，给 4KB 余量，放堆上不占任务栈。 */
    char *body = malloc(4096);
    if (!body) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = http_call("/commands", HTTP_METHOD_GET, NULL, body, 4096);
    if (err != ESP_OK) {
        free(body);
        return err;
    }

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    cJSON *ver = cJSON_GetObjectItem(root, "version");
    if (out_version && version_len) {
        strlcpy(out_version, cJSON_IsString(ver) ? ver->valuestring : "", version_len);
    }
    cJSON *list = cJSON_GetObjectItem(root, "commands");
    int n = 0;
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, list) {
        cJSON *id = cJSON_GetObjectItem(item, "id");
        cJSON *text = cJSON_GetObjectItem(item, "text");
        cJSON *ph = cJSON_GetObjectItem(item, "phonemes");
        if (cJSON_IsNumber(id) && cJSON_IsString(text) && cJSON_IsString(ph)) {
            cb(id->valueint, text->valuestring, ph->valuestring, ctx);
            n++;
        }
    }
    cJSON_Delete(root);
    ESP_LOGI(TAG, "从服务端拿到 %d 条命令词", n);
    return n > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t net_send_command(const char *text, char *reply, size_t reply_len, bool *out_ok,
                           bool *out_followup,
                           net_timing_t *timing)
{
    char req[192];
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "text", text);
    char *s = cJSON_PrintUnformatted(j);
    strlcpy(req, s, sizeof(req));
    cJSON_free(s);
    cJSON_Delete(j);

    char resp[512];
    /* 快路径用的是 perform()，上行只有一百来字节，拆不出有意义的 upload/wait ——
     * 整个往返记成 wait，服务端自报的分段仍然拿得到。 */
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = http_call("/command", HTTP_METHOD_POST, req, resp, sizeof(resp));
    if (err != ESP_OK) {
        return err;
    }
    if (timing) {
        timing->wait_ms = (int)((esp_timer_get_time() - t0) / 1000);
    }

    cJSON *root = cJSON_Parse(resp);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    read_server_ms(root, timing);
    if (out_ok) {
        *out_ok = cJSON_IsTrue(cJSON_GetObjectItem(root, "ok"));
    }
    if (out_followup) {
        *out_followup = cJSON_IsTrue(cJSON_GetObjectItem(root, "followup"));
    }
    cJSON *r = cJSON_GetObjectItem(root, "reply");
    if (reply && reply_len && cJSON_IsString(r)) {
        strlcpy(reply, r->valuestring, reply_len);
    }
    cJSON_Delete(root);
    return ESP_OK;
}

/* ---------------- 兜底路径（M4）----------------
 *
 * 这两个调用都不用 esp_http_client_perform()，而是手动 open/write/read。
 * 原因是两边的 body 都大到不该整段进内存：上行是几百 KB 的 PCM，
 * 下行是 TTS 的几十 KB —— 下行尤其要边收边播，收完再播白等半秒。
 */

/* 手动模式下把请求发出去并读到响应头。失败时负责 cleanup。 */
static esp_err_t http_open_write(esp_http_client_handle_t c,
                                 const void *body, size_t body_len)
{
    esp_err_t err = esp_http_client_open(c, body_len);
    if (err != ESP_OK) {
        return err;
    }
    const char *p = (const char *)body;
    size_t left = body_len;
    while (left > 0) {
        /* 一次写 4KB：再大 lwIP 的发送缓冲也吃不下，还会让这条任务长时间不让出 CPU。 */
        int n = esp_http_client_write(c, p, left > 4096 ? 4096 : (int)left);
        if (n <= 0) {
            return ESP_FAIL;
        }
        p += n;
        left -= n;
    }
    return esp_http_client_fetch_headers(c) < 0 ? ESP_FAIL : ESP_OK;
}

/* 同上，但把"写完 body"和"响应头到手"之间的时间单独记下来。 */
static esp_err_t http_open_write_split(esp_http_client_handle_t c,
                                       const void *body, size_t body_len,
                                       net_timing_t *timing)
{
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_open(c, body_len);
    if (err != ESP_OK) {
        return err;
    }
    const char *p = (const char *)body;
    size_t left = body_len;
    while (left > 0) {
        int n = esp_http_client_write(c, p, left > 4096 ? 4096 : (int)left);
        if (n <= 0) {
            return ESP_FAIL;
        }
        p += n;
        left -= n;
    }
    int64_t t1 = esp_timer_get_time();
    err = esp_http_client_fetch_headers(c) < 0 ? ESP_FAIL : ESP_OK;
    if (timing) {
        timing->upload_ms = (int)((t1 - t0) / 1000);
        timing->wait_ms = (int)((esp_timer_get_time() - t1) / 1000);
    }
    return err;
}

esp_err_t net_send_utterance(const void *pcm, size_t bytes,
                             char *text, size_t text_len,
                             char *reply, size_t reply_len, bool *out_ok,
                             net_timing_t *timing)
{
    if (!s_server_url[0]) {
        return ESP_ERR_INVALID_STATE;
    }
    char url[128];
    snprintf(url, sizeof(url), "%s/utterance", s_server_url);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        /* SenseVoice 在 Mac 上跑一句话大约 0.3s，但冷启动那次会久得多，给足 15s。 */
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return ESP_FAIL;
    }
    set_auth(c);
    esp_http_client_set_header(c, "Content-Type", "application/octet-stream");

    /* 手动模式在这儿有个额外好处：写完 body 和拿到响应头是两个可分的时刻，
     * 于是"上传花了多久"和"服务端算了多久"能分开量。perform() 只给得出总和。 */
    esp_err_t err = http_open_write_split(c, pcm, bytes, timing);
    char resp[512] = { 0 };
    if (err == ESP_OK) {
        int n = esp_http_client_read_response(c, resp, sizeof(resp) - 1);
        if (n < 0 || esp_http_client_get_status_code(c) != 200) {
            err = ESP_FAIL;
        } else {
            resp[n] = '\0';
        }
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "/utterance 失败: %s", esp_err_to_name(err));
        return err;
    }

    cJSON *root = cJSON_Parse(resp);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    read_server_ms(root, timing);
    cJSON *t = cJSON_GetObjectItem(root, "text");
    cJSON *r = cJSON_GetObjectItem(root, "reply");
    if (text && text_len && cJSON_IsString(t)) {
        strlcpy(text, t->valuestring, text_len);
    }
    if (reply && reply_len && cJSON_IsString(r)) {
        strlcpy(reply, r->valuestring, reply_len);
    }
    if (out_ok) {
        *out_ok = cJSON_IsTrue(cJSON_GetObjectItem(root, "ok"));
    }
    cJSON_Delete(root);
    return ESP_OK;
}

/* ---------------- 唤醒通知 ---------------- */

static void wake_task(void *arg)
{
    char url[128];
    snprintf(url, sizeof(url), "%s/wake", s_server_url);
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1500,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (c) {
        set_auth(c);
        esp_http_client_set_header(c, "Content-Type", "application/json");
        esp_http_client_set_post_field(c, "{}", 2);
        esp_http_client_perform(c);      /* 结果不关心：压音量失败不该影响识别 */
        esp_http_client_cleanup(c);
    }
    vTaskDelete(NULL);
}

void net_notify_wake(void)
{
    if (!s_server_url[0]) {
        return;
    }
    /* 单开一个任务而不是在回调里直接发：SR_EVENT_WAKE 是在 detect_task 里
     * 发出来的，那个任务每 32ms 就要喂一帧音频给 AFE，绝不能在里面做网络 IO。 */
    xTaskCreate(wake_task, "wake_notify", 4096, NULL, 4, NULL);
}

esp_err_t net_fetch_tts(const char *text, net_pcm_sink_t sink, void *ctx)
{
    if (!s_server_url[0] || !text || !text[0] || !sink) {
        return ESP_ERR_INVALID_ARG;
    }
    char url[128];
    snprintf(url, sizeof(url), "%s/tts", s_server_url);

    char req[256];
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "text", text);
    cJSON_AddStringToObject(j, "format", "pcm");   /* 要裸 PCM，省掉板上解 wav 头 */
    char *s = cJSON_PrintUnformatted(j);
    strlcpy(req, s, sizeof(req));
    cJSON_free(s);
    cJSON_Delete(j);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        return ESP_FAIL;
    }
    set_auth(c);
    esp_http_client_set_header(c, "Content-Type", "application/json");

    esp_err_t err = http_open_write(c, req, strlen(req));
    if (err == ESP_OK && esp_http_client_get_status_code(c) != 200) {
        err = ESP_FAIL;
    }
    if (err == ESP_OK) {
        /* 4KB = 2048 采样 = 128ms 音频。够大不至于让 codec 饿着，
         * 又够小让第一包很快到手。 */
        char *buf = malloc(4096);
        if (!buf) {
            err = ESP_ERR_NO_MEM;
        } else {
            int n;
            while ((n = esp_http_client_read(c, buf, 4096)) > 0) {
                if (sink(buf, n, ctx) != ESP_OK) {
                    break;
                }
            }
            if (n < 0) {
                err = ESP_FAIL;
            }
            free(buf);
        }
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "/tts 失败: %s", esp_err_to_name(err));
    }
    return err;
}
