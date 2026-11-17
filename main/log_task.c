/*
 * 日志记录任务 —— 单文件、双流（遥测 + 事件），按天分文件
 *
 * 为什么这么设计（与需求对齐）：
 *   1. 遥测和事件写进【同一个文件】，时间轴天然对齐，
 *      "温度变化 -> HEAT_ON -> 温度回升" 的因果关系在文件里连续可见。
 *   2. 用第 3 列 kind 区分行类型：
 *        T    = 周期遥测（每 5 秒）
 *        E    = 事件（执行器动作 / 模式切换 / 故障 / 链路）
 *        LOST = UART 断链，时间戳照常盖，值字段填 LINKLOST
 *   3. 执行器动作靠【对比 STM32 遥测的变化】得出，不改 STM32 固件。
 *   4. 优先级最低（2），并且录像期间自动放宽采样周期，把 SD 带宽让给录像。
 *      优先级链：以太网视频流 > 视频留存 > 日志
 *
 * 文件：/sdcard/logs/YYYYMMDD.csv
 * 表头：ts,ms,kind,name,tin,hin,tout,hout,ah_in,td_in,tset,rhset,mode,cool,heat,hum,light,fan,tfan,fault
 */

#include "log_task.h"
#include "video_record_task.h"
#include "uart_comm_task.h"
#include "common.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "LOG_TASK";

/* 其它任务（AI 决策、HTTP）往里丢事件，由日志任务统一落盘 */
static QueueHandle_t s_evt_q;

#define LOG_DIR        "/sdcard/logs"
#define LOG_PERIOD_MS      5000                      /* 平时 5 秒一行 */
#define LOG_PERIOD_REC_MS  10000                     /* 录像时放宽到 10 秒 */

static const char *CSV_HEADER =
    "ts,ms,kind,name,tin,hin,tout,hout,ah_in,td_in,tset,rhset,"
    "mode,cool,heat,hum,light,fan,tfan,fault\n";

typedef struct {
    FILE       *fp;
    int         day_key;        /* tm_yday，用于跨天换文件 */
    bool        have_prev;
    stm_state_t prev;
    bool        prev_link;
    bool        warned_clock;
} log_ctx_t;

/* ------------------------------------------------------------------ */
/* 基础工具                                                            */
/* ------------------------------------------------------------------ */

static void now_iso(char *buf, size_t n, struct tm *tm_out)
{
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    if (tm_out) {
        *tm_out = tm;
    }
    strftime(buf, n, "%Y-%m-%dT%H:%M:%S", &tm);
}

static bool clock_sane(const struct tm *tm)
{
    return (tm->tm_year + 1900) >= 2020;
}

static void ensure_file(log_ctx_t *c, const struct tm *tm)
{
    if (c->fp != NULL && c->day_key == tm->tm_yday) {
        return;
    }
    if (c->fp) {
        fclose(c->fp);
        c->fp = NULL;
    }

    char path[64];
    snprintf(path, sizeof(path), LOG_DIR "/%04d%02d%02d.csv",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);

    struct stat fst;
    bool is_new = (stat(path, &fst) != 0) || (fst.st_size == 0);

    FILE *f = fopen(path, "a");
    if (!f) {
        ESP_LOGE(TAG, "打开日志失败: %s (errno=%d)", path, errno);
        return;
    }
    static char iobuf[4096];
    setvbuf(f, iobuf, _IOFBF, sizeof(iobuf));

    if (is_new) {
        fprintf(f, "# silkworm log v1 boot=%d\n", (int)esp_reset_reason());
        fputs(CSV_HEADER, f);
        fflush(f);
        fsync(fileno(f));
        ESP_LOGI(TAG, "新建日志文件 %s", path);
    }
    c->day_key = tm->tm_yday;
    c->fp = f;
}

/* ------------------------------------------------------------------ */
/* 写一行                                                              */
/* ------------------------------------------------------------------ */

static void write_telemetry(log_ctx_t *c, const stm_state_t *s)
{
    char ts[24];
    struct tm tm;
    int64_t ms = esp_timer_get_time() / 1000;
    now_iso(ts, sizeof(ts), &tm);
    ensure_file(c, &tm);
    if (!c->fp) {
        return;
    }

    if (!clock_sane(&tm) && !c->warned_clock) {
        c->warned_clock = true;
        ESP_LOGW(TAG, "系统时间未同步（SNTP 未成功），ts 列会显示 1970 年；"
                      "请以 ms 列为准");
    }

    if (s->link_ok) {
        fprintf(c->fp,
                "%s,%lld,T,,%.2f,%.1f,%.2f,%.1f,%.2f,%.2f,%.1f,%.1f,"
                "%s,%d,%d,%d,%d,%d,%d,%d\n",
                ts, (long long)ms,
                (double)s->tin, (double)s->hin, (double)s->tout, (double)s->hout,
                (double)s->ah_in, (double)s->td_in,
                (double)s->t_set, (double)s->rh_set,
                s->mode, s->cool, s->heat, s->hum,
                s->light, s->fan, s->tec_fan, s->fault);
    } else {
        /* 断链：时间戳照常盖，值字段填 LINKLOST（需求指定） */
        fprintf(c->fp, "%s,%lld,LOST,,", ts, (long long)ms);
        for (int i = 0; i < 16; i++) {
            fputs("LINKLOST", c->fp);
            fputc(i == 15 ? '\n' : ',', c->fp);
        }
    }
    fflush(c->fp);
    fsync(fileno(c->fp));      /* 关键：更新目录项大小，断电后文件才可见 */
}

static void write_event(log_ctx_t *c, const char *name)
{
    char ts[24];
    struct tm tm;
    int64_t ms = esp_timer_get_time() / 1000;
    now_iso(ts, sizeof(ts), &tm);
    ensure_file(c, &tm);
    if (!c->fp) {
        return;
    }

    fprintf(c->fp, "%s,%lld,E,%s", ts, (long long)ms, name);
    for (int i = 0; i < 16; i++) {
        fputc(',', c->fp);
    }
    fputc('\n', c->fp);
    fflush(c->fp);
    fsync(fileno(c->fp));

    ESP_LOGI(TAG, "事件 %s", name);
}

/* ------------------------------------------------------------------ */
/* 事件生成：对比上一帧遥测                                            */
/* ------------------------------------------------------------------ */

static void detect_events(log_ctx_t *c, const stm_state_t *s)
{
    char name[48];

    if (!s->link_ok) {
        if (c->prev_link) {
            write_event(c, "LINK_LOST");
            c->prev_link = false;
            c->have_prev = false;      /* 恢复后重建基准，避免误报 */
        }
        return;
    }
    if (!c->prev_link) {
        write_event(c, "LINK_UP");
        c->prev_link = true;
    }
    if (!c->have_prev) {
        c->prev = *s;
        c->have_prev = true;
        return;
    }

    const stm_state_t *p = &c->prev;

    if (strcmp(p->mode, s->mode) != 0) {
        snprintf(name, sizeof(name), "MODE_%s", s->mode);
        write_event(c, name);
    }
    if (p->heat != s->heat) {
        if (s->heat) {
            snprintf(name, sizeof(name), "HEAT_ON_%d", s->heat);
        } else {
            snprintf(name, sizeof(name), "HEAT_OFF");
        }
        write_event(c, name);
    }
    if (p->cool != s->cool) {
        if (s->cool) {
            snprintf(name, sizeof(name), "COOL_ON_%d", s->cool);
        } else {
            snprintf(name, sizeof(name), "COOL_OFF");
        }
        write_event(c, name);
    }
    if (p->hum != s->hum) {
        write_event(c, s->hum ? "HUM_ON" : "HUM_OFF");
    }
    if (p->light != s->light) {
        write_event(c, s->light ? "LIGHT_ON" : "LIGHT_OFF");
    }
    if (p->fan != s->fan) {
        write_event(c, s->fan ? "FAN_ON" : "FAN_OFF");
    }
    if (p->tec_fan != s->tec_fan) {
        write_event(c, s->tec_fan ? "TFAN_ON" : "TFAN_OFF");
    }
    if (p->fault != s->fault) {
        snprintf(name, sizeof(name), "FAULT_%d", s->fault);
        write_event(c, name);
    }

    c->prev = *s;
}

/* ------------------------------------------------------------------ */
/* 主循环                                                              */
/* ------------------------------------------------------------------ */

static void log_write_loop(void *arg)
{
    log_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.day_key = -1;

    ESP_LOGI(TAG, "日志任务启动（周期 %d ms，录像时 %d ms）",
             LOG_PERIOD_MS, LOG_PERIOD_REC_MS);

    write_event(&c, "BOOT");

    int64_t next_sample = esp_timer_get_time() / 1000 + LOG_PERIOD_MS;

    while (1) {
        /* 每 100ms 看一次有没有外部事件要写（AI 决策等），有就立刻落盘 */
        char evt[48];
        if (xQueueReceive(s_evt_q, evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            write_event(&c, evt);
        }

        int64_t now = esp_timer_get_time() / 1000;
        if (now >= next_sample) {
            int period = video_is_recording() ? LOG_PERIOD_REC_MS : LOG_PERIOD_MS;
            next_sample = now + period;

            stm_state_t s;
            stm_get_state(&s);

            detect_events(&c, &s);
            write_telemetry(&c, &s);
        }
    }
}

/* 供其它任务写入一条事件（不会阻塞，队列满就丢） */
void log_task_post_event(const char *name)
{
    if (s_evt_q == NULL || name == NULL) {
        return;
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "%s", name);
    xQueueSend(s_evt_q, buf, 0);
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

void log_task_init(void)
{
    ESP_LOGI(TAG, "初始化日志任务...");

    struct stat st;
    if (stat("/sdcard", &st) != 0) {
        ESP_LOGW(TAG, "SD 未挂载，日志任务禁用");
        return;
    }
    if (stat(LOG_DIR, &st) != 0) {
        if (mkdir(LOG_DIR, 0755) == 0) {
            ESP_LOGI(TAG, "已创建 %s", LOG_DIR);
        }
    }

    s_evt_q = xQueueCreate(16, 48);
    if (s_evt_q == NULL) {
        ESP_LOGE(TAG, "事件队列创建失败");
        return;
    }

    /* 优先级 2：低于录像(4)、摄像头、HTTP，保证录像优先拿到 SD 带宽 */
    xTaskCreatePinnedToCore(log_write_loop, "log_write",
                            6144, NULL, 2, NULL, 0);

    ESP_LOGI(TAG, "[OK] 日志任务已启动");
}
