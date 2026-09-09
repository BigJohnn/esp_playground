#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_codec_dev.h"
#include "esp_err.h"

typedef enum {
    SR_EVENT_WAKE,        /* 听到唤醒词 */
    SR_EVENT_COMMAND,     /* 唤醒后识别出命令词，text 是对应的中文句子 */
    SR_EVENT_UTTERANCE,   /* 唤醒后说了话，但不在命令词表里 —— pcm 里是这整段录音 */
    SR_EVENT_TIMEOUT,     /* 唤醒后压根没出声 */
} sr_event_t;

typedef struct {
    sr_event_t event;
    int command_id;       /* 仅 SR_EVENT_COMMAND */
    const char *text;     /* 仅 SR_EVENT_COMMAND */
    float prob;           /* 仅 SR_EVENT_COMMAND */
    const int16_t *pcm;   /* 仅 SR_EVENT_UTTERANCE：16k/16bit 单声道，AFE 处理后的 */
    size_t samples;       /* 仅 SR_EVENT_UTTERANCE */
} sr_result_t;

typedef void (*sr_event_cb_t)(const sr_result_t *res, void *ctx);

/* 注册一条命令词。必须在 sr_start() 之前调用。
 *   text     中文原文，识别到之后原样发给服务端
 *   phonemes 无声调拼音、按音节空格分隔，例如 "da kai tai deng" —— mn7_cn 的输入单位 */
esp_err_t sr_add_command(int id, const char *text, const char *phonemes);
int sr_command_count(void);

/* 起 AFE + WakeNet + MultiNet，两条常驻任务（喂数据 / 取结果）。 */
esp_err_t sr_start(esp_codec_dev_handle_t codec, sr_event_cb_t cb, void *ctx);

/* SR_EVENT_UTTERANCE 里给出的 pcm 一直有效到这次调用为止，期间不会再录新的。
 * 用完必须还 —— 否则兜底路径就此哑掉。 */
void sr_release_utterance(void);

/* ---- 命令词表的热更新 ----
 *
 * 词表在服务端，板子开机拉一次；服务端改了词之后不该还要重启板子（重启要 4 秒
 * 重建模型，还得够得着串口）。所以这里做成"暂存 -> 提交"：
 * 网络任务往暂存表里写，detect 任务在两次唤醒之间的空档把它换上去。
 * 不直接改在用的那张表 —— detect 任务正拿着 id 去查文本，写到一半会读出乱码。 */
void sr_commands_begin(void);
void sr_commands_commit(void);

/* ---- DAC 回环探针（没喇叭时验证播放链路用）----
 *
 * ES8311 把 DAC 数据镜像回 I2S 输入的右声道，所以"板子送出去了什么"是可测的，
 * 不必依赖喇叭。start 之后放一段音频，stop 拿结果。
 * tone_amp 是指定频点上的幅度（和 int16 满量程 32767 同尺度）。 */
typedef struct {
    float ref_rms;    /* 右声道（DAC 回环）RMS */
    float mic_rms;    /* 左声道（麦克风）RMS，同期对照 */
    float tone_amp;   /* 右声道在探测频点上的幅度 */
    int   samples;
} sr_ref_probe_t;

void sr_ref_probe_start(int freq_hz);
void sr_ref_probe_stop(sr_ref_probe_t *out);

/* 放 TTS 的时候要把麦克风静音。板上是单颗模拟麦、喇叭信号也没接回 AFE，
 * 没有 AEC 可用，不静音就会听见自己说话（轻则误唤醒，重则自己跟自己聊）。 */
void sr_set_muted(bool muted);
