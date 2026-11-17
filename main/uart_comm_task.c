/*
 * UART 通信任务 —— ESP32-P4 ↔ STM32F103C8T6
 *
 * 协议严格按交接文档 §5：
 *   - 一行一条 JSON，'\n' 结尾，115200 8N1，无 CRC、无包头
 *   - STM32 → P4：每 1s 一帧 telemetry；应答 pong / ack / version / post
 *   - P4 → STM32：{"mode":"manual"} 之类命令，冒号后不能有空格
 *
 * 相比旧骨架新增：
 *   1. 解析真实遥测字段（type=telemetry）
 *   2. 半帧超时（50ms 收不到 '\n' 就清缓冲）—— 文档 §7 第 3 条
 *   3. 心跳 ping/pong + 遥测新鲜度判在线
 *   4. 遥测快照供网页读取
 */

#include "uart_comm_task.h"
#include "common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "STM32";

#define UART_NUM         CONFIG_UART_STM32_NUM
#define UART_TX_PIN      CONFIG_UART_STM32_TX_PIN
#define UART_RX_PIN      CONFIG_UART_STM32_RX_PIN
#define UART_BAUD_RATE   CONFIG_UART_STM32_BAUD_RATE

#define RX_LINE_MAX      640
#define HALF_FRAME_MS    50        /* 文档 §7-3：半帧超时 */
#define LINK_TIMEOUT_MS  3500      /* 3.5s 没遥测就判离线 */
#define PING_PERIOD_MS   5000      /* 文档 §6-6：5s 一次 ping */

float latest_temp = 0.0f;
float latest_humidity = 0.0f;

static stm_state_t        s_st;
static SemaphoreHandle_t  s_lock;
static volatile int64_t   s_last_tm_us = 0;
static bool               s_was_online = false;

/* ---------------- 极简 JSON 取值（不引 cJSON） ---------------- */

static const char *find_key(const char *s, const char *key)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    return strstr(s, pat);
}

static bool jnum(const char *s, const char *key, float *out)
{
    const char *p = find_key(s, key);
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p++;
    while (*p == ' ') {
        p++;
    }
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

static int jint(const char *s, const char *key, int dflt)
{
    float v;
    return jnum(s, key, &v) ? (int)v : dflt;
}

static bool jstr(const char *s, const char *key, char *out, size_t cap)
{
    const char *p = find_key(s, key);
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p = strchr(p, '"');
    if (!p) {
        return false;
    }
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) {
        out[i++] = *p++;
    }
    out[i] = 0;
    return i > 0;
}

/* ---------------- 帧处理 ---------------- */

static void handle_line(const char *line)
{
    char type[16] = { 0 };
    jstr(line, "type", type, sizeof(type));

    if (strcmp(type, "telemetry") == 0) {
        xSemaphoreTake(s_lock, portMAX_DELAY);

        s_st.ms  = (uint32_t)jint(line, "ms", 0);
        s_st.seq = (uint32_t)jint(line, "seq", 0);

        jnum(line, "tin",  &s_st.tin);
        jnum(line, "hin",  &s_st.hin);
        jnum(line, "tout", &s_st.tout);
        jnum(line, "hout", &s_st.hout);
        jnum(line, "ah_in",  &s_st.ah_in);
        jnum(line, "ah_out", &s_st.ah_out);
        jnum(line, "td_in",  &s_st.td_in);
        jnum(line, "td_out", &s_st.td_out);
        jnum(line, "ah_set", &s_st.ah_set);
        jnum(line, "t_set",  &s_st.t_set);
        jnum(line, "rh_set", &s_st.rh_set);

        s_st.in_valid  = jint(line, "in_valid", 0);
        s_st.out_valid = jint(line, "out_valid", 0);
        jstr(line, "mode", s_st.mode, sizeof(s_st.mode));

        s_st.cool      = jint(line, "cool", 0);
        s_st.cool_duty = jint(line, "cool_duty", 0);
        s_st.heat      = jint(line, "heat", 0);
        s_st.heat_duty = jint(line, "heat_duty", 0);
        s_st.hum       = jint(line, "hum", 0);
        s_st.light     = jint(line, "light", 0);
        s_st.light_level = jint(line, "light_level", 0);
        s_st.fan       = jint(line, "fan", 0);
        s_st.tec_fan   = jint(line, "tec_fan", 0);
        s_st.fault     = jint(line, "fault", 0);

        s_st.rx_frames++;
        s_st.link_ok = true;

        latest_temp     = s_st.tin;
        latest_humidity = s_st.hin;
        s_last_tm_us    = esp_timer_get_time();

        xSemaphoreGive(s_lock);

        if (!s_was_online) {
            s_was_online = true;
            ESP_LOGI(TAG, "[LINK] STM32 上线  tin=%.1f°C hin=%.0f%% mode=%s",
                     (double)s_st.tin, (double)s_st.hin, s_st.mode);
        }
        return;
    }

    if (strcmp(type, "pong") == 0) {
        ESP_LOGD(TAG, "pong %s", line);
        return;
    }
    if (strcmp(type, "version") == 0) {
        char fw[24] = { 0 };
        jstr(line, "fw", fw, sizeof(fw));
        ESP_LOGI(TAG, "STM32 固件: %s", fw);
        return;
    }
    if (strcmp(type, "ack") == 0) {
        char cmd[16] = { 0 };
        jstr(line, "cmd", cmd, sizeof(cmd));
        ESP_LOGI(TAG, "ack cmd=%s ok=%d", cmd, jint(line, "ok", 1));
        return;
    }
    if (strcmp(type, "post") == 0 || strcmp(type, "i2c") == 0) {
        ESP_LOGI(TAG, "自检帧: %s", line);
        return;
    }

    /* 未知帧：留着看，别丢信息 */
    ESP_LOGW(TAG, "未识别帧: %s", line);
}


/* 在把引脚交给 UART 之前，先当普通输入采样一下电平。
 * 恒为 0 → 线被拉低/没接/接错；恒为 1 → 空闲高电平（UART 正常空闲态） */
static void rx_line_probe(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << UART_RX_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* 采样 1.2 秒：STM32 每 1000ms 发一帧约 4.5ms 的突发，
     * 10ms 窗口撞上的概率只有 0.45%，必须拉长到 1.2s 才可靠。 */
    const int N = 24000;            /* 24000 x 50us = 1.2s */
    int high = 0, edges = 0;
    int prev = gpio_get_level((gpio_num_t)UART_RX_PIN);

    for (int i = 0; i < N; i++) {
        int v = gpio_get_level((gpio_num_t)UART_RX_PIN);
        if (v != prev) {
            edges++;
            prev = v;
        }
        if (v) {
            high++;
        }
        esp_rom_delay_us(50);
    }

    if (edges >= 20) {
        ESP_LOGI(TAG, "RX(GPIO%d) 1.2s 采样: 高=%d/%d 跳变=%d → 线上有数据在跑",
                 UART_RX_PIN, high, N, edges);
    } else if (edges > 0) {
        ESP_LOGW(TAG, "RX(GPIO%d) 1.2s 采样: 高=%d/%d 跳变=%d → 只有零星跳变，"
                 "疑似干扰或波特率严重不匹配", UART_RX_PIN, high, N, edges);
    } else {
        if (high == N) {
            ESP_LOGE(TAG, "RX(GPIO%d) 1.2s 采样: 恒为高、跳变=0 → "
                     "线上完全没有信号（对方没发 / 线没接到 PA2 / 接错引脚）",
                     UART_RX_PIN);
        } else {
            ESP_LOGE(TAG, "RX(GPIO%d) 1.2s 采样: 恒为低、跳变=0 → 线被拉低或接到 GND",
                     UART_RX_PIN);
        }
    }
}

/* 内部回环自测：不需要任何接线，验证 UART 驱动与收包路径 */
static volatile bool s_want_loop   = false;
static volatile int  s_loop_result = 0;   /* 0=未测 1=成功 2=失败 */

void uart_comm_request_loopback(void)
{
    s_want_loop = true;
}

static void run_loopback_test(void)
{
    const char *msg = "LOOP_TEST_123";
    uint8_t rx[32] = { 0 };

    uart_set_loop_back(UART_NUM, true);
    uart_flush_input(UART_NUM);
    uart_write_bytes(UART_NUM, msg, strlen(msg));
    int n = uart_read_bytes(UART_NUM, rx, sizeof(rx) - 1, pdMS_TO_TICKS(300));
    uart_set_loop_back(UART_NUM, false);

    if (n > 0) {
        rx[n] = 0;
        s_loop_result = 1;
        ESP_LOGI(TAG, "[回环] 成功收到 %d 字节: %s", n, (char *)rx);
    } else {
        s_loop_result = 2;
        ESP_LOGE(TAG, "[回环] 失败：P4 的 UART 收发路径本身有问题");
    }
}


/* ---------- 原始行环形缓冲（诊断用：解析失败时也能看到对方发了什么） ---------- */
#define RAW_LINES 8
#define RAW_LEN   200
static char     s_raw[RAW_LINES][RAW_LEN];
static volatile int      s_raw_head  = 0;
static volatile uint32_t s_raw_total = 0;

static void raw_store(const char *src)
{
    int k = 0;
    for (; *src && k < RAW_LEN - 1; src++) {
        unsigned char c = (unsigned char)*src;
        if (c >= 0x20 && c < 0x7F) {
            s_raw[s_raw_head][k++] = (char)c;
        }
    }
    s_raw[s_raw_head][k] = 0;
    s_raw_head = (s_raw_head + 1) % RAW_LINES;
    s_raw_total++;
}

int uart_comm_get_raw(char *out, int max)
{
    int off = 0;
    int start = (s_raw_head - RAW_LINES + RAW_LINES) % RAW_LINES;
    for (int i = 0; i < RAW_LINES; i++) {
        const char *ln = s_raw[(start + i) % RAW_LINES];
        if (ln[0] == 0) {
            continue;
        }
        int len = (int)strlen(ln);
        if (off + len + 2 >= max) {
            break;
        }
        memcpy(out + off, ln, len);
        off += len;
        out[off++] = '\n';
    }
    out[off] = 0;
    return off;
}

int uart_comm_loop_result(void)
{
    return s_loop_result;
}

uint32_t uart_comm_raw_total(void)
{
    return s_raw_total;
}

/* ---------------- 收 ---------------- */

static void rx_task(void *arg)
{
    uint8_t buf[256];
    char    line[RX_LINE_MAX];
    int     n = 0;
    int64_t last_byte = 0;

    while (1) {
        if (s_want_loop) {
            s_want_loop = false;
            run_loopback_test();
        }

        int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        int64_t now = esp_timer_get_time() / 1000;

        if (len > 0) {
            last_byte = now;
            for (int i = 0; i < len; i++) {
                char c = (char)buf[i];
                if (c == '\n' || c == '\r') {
                    if (n > 0) {
                        line[n] = 0;
                        raw_store(line);
                        if (s_raw_total <= 3) {
                            ESP_LOGI(TAG, "首帧原始内容[%u]: %s",
                                     (unsigned)s_raw_total, line);
                        }
                        handle_line(line);
                        n = 0;
                    }
                } else if (n < RX_LINE_MAX - 1) {
                    line[n++] = c;
                } else {
                    n = 0;      /* 超长帧直接丢 */
                }
            }
        } else if (n > 0 && (now - last_byte) > HALF_FRAME_MS) {
            /* 半帧超时：对方掉电或干扰，缓冲里的残帧必须丢掉 */
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_st.half_frames++;
            xSemaphoreGive(s_lock);
            n = 0;
            ESP_LOGW(TAG, "半帧超时，清空接收缓冲");
        }
    }
}

/* ---------------- 心跳 / 在线判定 ---------------- */

static void hb_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* 上电先问版本和状态，确认链路 */
    uart_send_to_stm32("{\"version\":1}");
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_send_to_stm32("{\"status\":1}");

    int64_t last_ping = esp_timer_get_time() / 1000;
    int64_t start_ms  = last_ping;
    int64_t last_sum  = 0;
    bool    hint_done = false;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        if (now - last_ping >= PING_PERIOD_MS) {
            last_ping = now;
            uart_send_to_stm32("{\"ping\":1}");
        }

        /* 开机 6 秒还没收到任何帧 → 打一份排查清单，省得来回问 */
        if (!hint_done && (now - start_ms) > 6000) {
            hint_done = true;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            uint32_t rx = s_st.rx_frames;
            uint32_t half = s_st.half_frames;
            xSemaphoreGive(s_lock);

            if (rx == 0 && half == 0) {
                ESP_LOGW(TAG, "6 秒内一帧都没收到，按顺序查：");
                ESP_LOGW(TAG, "  1) 交叉接线：P4 GPIO22(TX) -> STM32 PA3(RX)");
                ESP_LOGW(TAG, "  2) 交叉接线：P4 GPIO21(RX) <- STM32 PA2(TX)");
                ESP_LOGW(TAG, "  3) 地是否连通：P4 GND ↔ STM32 GND");
                ESP_LOGW(TAG, "  4) STM32 是否在发：USART1 上应能看到每秒一行 telemetry");
            } else if (rx == 0 && half > 0) {
                ESP_LOGW(TAG, "收到数据但全是半帧（%u 次）→ 波特率或线路质量有问题",
                         (unsigned)half);
            } else {
                ESP_LOGI(TAG, "链路正常，已收 %u 帧", (unsigned)rx);
            }
        }

        /* 每 10 秒打一条遥测摘要，不开网页也能看 */
        if (now - last_sum >= 10000) {
            last_sum = now;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            stm_state_t s = s_st;
            xSemaphoreGive(s_lock);
            if (s.rx_frames > 0) {
                ESP_LOGI(TAG, "遥测 seq=%u 箱内 %.1f°C/%.0f%% 箱外 %.1f°C/%.0f%% "
                         "露点 %.1f°C AH %.2f 设定 %.1f°C/%.0f%% 模式=%s "
                         "制冷%d 加热%d 加湿%d 补光%d 故障=%d",
                         (unsigned)s.seq, (double)s.tin, (double)s.hin,
                         (double)s.tout, (double)s.hout, (double)s.td_in,
                         (double)s.ah_in, (double)s.t_set, (double)s.rh_set,
                         s.mode, s.cool, s.heat, s.hum, s.light, s.fault);
            }
        }

        bool online = (s_last_tm_us != 0) &&
                      ((esp_timer_get_time() - s_last_tm_us) / 1000 < LINK_TIMEOUT_MS);

        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_st.link_ok = online;
        if (s_last_tm_us) {
            s_st.age_ms = (uint32_t)((esp_timer_get_time() - s_last_tm_us) / 1000);
        }
        xSemaphoreGive(s_lock);

        if (!online && s_was_online) {
            s_was_online = false;
            ESP_LOGW(TAG, "[LINK] STM32 离线（>%d ms 无遥测）", LINK_TIMEOUT_MS);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------------- 对外接口 ---------------- */

void uart_send_to_stm32(const char *json_str)
{
    if (json_str == NULL) {
        return;
    }
    uart_write_bytes(UART_NUM, json_str, strlen(json_str));
    uart_write_bytes(UART_NUM, "\n", 1);
    ESP_LOGD(TAG, "TX> %s", json_str);
}

void stm_get_state(stm_state_t *out)
{
    if (out == NULL) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_st;
    xSemaphoreGive(s_lock);
}

void uart_comm_task_init(void)
{
    ESP_LOGI(TAG, "初始化 UART%d (TX=%d, RX=%d, %d 8N1) ↔ STM32 USART2 (PA2/PA3)",
             UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);

    /* P4 的 console 占 GPIO37(TX)/38(RX)。RX 若选 38 会把 console 接收端抢走，
     * 表现是串口监视器只能看不能打字。功能上能收 STM32 数据，但要知道这个副作用。 */
    if (UART_RX_PIN == 38 || UART_RX_PIN == 37) {
        ESP_LOGW(TAG, "注意：GPIO%d 是 console UART 引脚，改作 UART%d RX 后"
                      "串口监视器将无法输入（输出不受影响）", UART_RX_PIN, UART_NUM);
    }

    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    rx_line_probe();

    /* 顺序必须按 IDF 文档：先 param_config → 再 set_pin → 最后 driver_install */
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 4096, 1024, 0, NULL, 0));

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return;
    }
    memset(&s_st, 0, sizeof(s_st));
    strcpy(s_st.mode, "?");

    xTaskCreatePinnedToCore(rx_task, "stm_rx", 4096, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(hb_task, "stm_hb", 3072, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "[OK] STM32 链路就绪");
}
