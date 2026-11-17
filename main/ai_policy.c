/*
 * AI 决策模块（P4 侧）
 *
 * 分工：
 *   PC (YOLO11)  → 只告诉我们「是什么」：蚕数、病蚕数、异动、置信度
 *   P4 (本模块)  → 决定「怎么办」：查表 → 变成 STM32 的现有命令
 *   STM32        → 执行，并继续环境闭环保底
 *
 * 设计原则（演示够用、别把箱子搞坏）：
 *   1. 置信度 < 0.5 一律不动作，只记日志（防误判）
 *   2. 异动要连续 3 帧超阈值才动作（防抖）
 *   3. 任何动作后有 120 秒冷却（防止来回抖）
 *   4. 只使用 STM32 已有的命令：set_temp / set_humi / light_level / fan
 *   5. 每次决策都写进日志（AI_xxx），能和执行器反馈对上，证明闭环
 */

#include "ai_policy.h"
#include "uart_comm_task.h"
#include "log_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "AI";

/* ---- 阈值（想调就改这里） ---- */
#define CONF_MIN        0.50f     /* 低于此置信度不动作 */
#define SICK_MILD       0.05f     /* 5% 病蚕 → 轻微干预 */
#define SICK_SEVERE     0.15f     /* 15% → 加强干预 */
#define MOTION_HI       0.30f     /* 异动指数阈值 */
#define MOTION_HITS     3         /* 连续几帧算数 */
#define COOLDOWN_MS     120000    /* 动作冷却 2 分钟 */
#define FAN_BOOST_MS    300000    /* 强通风持续 5 分钟 */
#define EMPTY_MS        60000     /* 空箱持续 60 秒后关灯 */
#define STALE_MS        30000     /* 30 秒没新结果 = 视觉离线 */

static ai_result_t s_last;
static int64_t  s_last_us;
static bool     s_enabled        = true;
static int      s_motion_hits;
static int64_t  s_cooldown_until;
static int64_t  s_fan_off_at;
static int64_t  s_empty_since;
static bool     s_empty_done;
static bool     s_stale_warned;
static char     s_last_action[64] = "-";
static int      s_actions;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/* 下发一条命令 + 写一条决策日志 */
static void act(const char *json, const char *event)
{
    uart_send_to_stm32(json);
    log_task_post_event(event);
    snprintf(s_last_action, sizeof(s_last_action), "%s", event);
    s_actions++;
    ESP_LOGI(TAG, "决策 %s -> %s", event, json);
}

/* ------------------------------------------------------------------ */
/* 规则表                                                              */
/* ------------------------------------------------------------------ */

static void act_sick_mild(void)
{
    stm_state_t st;
    stm_get_state(&st);

    float t = (st.t_set > 1.0f ? st.t_set : 24.5f) + 0.5f;
    float h = (st.rh_set > 1.0f ? st.rh_set : 70.0f) - 5.0f;
    if (h < 50.0f) {
        h = 50.0f;
    }

    char cmd[48];
    snprintf(cmd, sizeof(cmd), "{\"set_humi\":%.0f}", (double)h);
    uart_send_to_stm32(cmd);
    snprintf(cmd, sizeof(cmd), "{\"set_temp\":%.1f}", (double)t);
    uart_send_to_stm32(cmd);
    uart_send_to_stm32("{\"light_level\":100}");

    snprintf(cmd, sizeof(cmd), "AI_SICK_MILD_%d", (int)(s_last.sick_ratio * 100.0f));
    log_task_post_event(cmd);
    snprintf(s_last_action, sizeof(s_last_action), "%s", cmd);
    s_actions++;
    ESP_LOGI(TAG, "决策 病蚕%.0f%% -> 降湿到 %.0f%%、升温到 %.1f°C、补光100%%",
             (double)(s_last.sick_ratio * 100.0f), (double)h, (double)t);
}

static void act_sick_severe(void)
{
    stm_state_t st;
    stm_get_state(&st);

    float t = (st.t_set > 1.0f ? st.t_set : 24.5f) + 0.5f;
    float h = (st.rh_set > 1.0f ? st.rh_set : 70.0f) - 8.0f;
    if (h < 45.0f) {
        h = 45.0f;
    }

    char cmd[48];
    snprintf(cmd, sizeof(cmd), "{\"set_humi\":%.0f}", (double)h);
    uart_send_to_stm32(cmd);
    snprintf(cmd, sizeof(cmd), "{\"set_temp\":%.1f}", (double)t);
    uart_send_to_stm32(cmd);
    uart_send_to_stm32("{\"light_level\":128}");
    uart_send_to_stm32("{\"fan\":1}");

    s_fan_off_at = now_ms() + FAN_BOOST_MS;

    snprintf(cmd, sizeof(cmd), "AI_SICK_SEVERE_%d", (int)(s_last.sick_ratio * 100.0f));
    log_task_post_event(cmd);
    snprintf(s_last_action, sizeof(s_last_action), "%s", cmd);
    s_actions++;
    ESP_LOGW(TAG, "决策 病蚕%.0f%% -> 降湿到 %.0f%%、升温到 %.1f°C、强通风5分钟",
             (double)(s_last.sick_ratio * 100.0f), (double)h, (double)t);
}

static void act_motion(void)
{
    uart_send_to_stm32("{\"light_level\":50}");
    log_task_post_event("AI_MOTION_REST");
    snprintf(s_last_action, sizeof(s_last_action), "AI_MOTION_REST");
    s_actions++;
    ESP_LOGI(TAG, "决策 异动%.2f 连续%d帧 -> 补光降到50%%",
             (double)s_last.motion, s_motion_hits);
}

/* ------------------------------------------------------------------ */
/* 提交推理结果                                                        */
/* ------------------------------------------------------------------ */

void ai_policy_submit(const ai_result_t *r)
{
    s_last = *r;
    if (s_last.sick_ratio <= 0.0f && s_last.sick > 0 && s_last.count > 0) {
        s_last.sick_ratio = (float)s_last.sick / (float)s_last.count;
    }
    s_last_us = esp_timer_get_time();
    s_stale_warned = false;

    if (!s_enabled) {
        return;
    }

    /* 1) 置信度门槛 */
    if (s_last.conf >= 0.0f && s_last.conf < CONF_MIN) {
        log_task_post_event("AI_LOWCONF");
        ESP_LOGW(TAG, "置信度 %.2f 过低，不动作", (double)s_last.conf);
        return;
    }

    /* 2) 连续帧计数 */
    if (s_last.motion > MOTION_HI) {
        s_motion_hits++;
    } else {
        s_motion_hits = 0;
    }

    /* 3) 空箱计时 */
    if (s_last.count == 0) {
        if (s_empty_since == 0) {
            s_empty_since = now_ms();
        }
    } else {
        s_empty_since = 0;
        s_empty_done  = false;
    }

    /* 4) 冷却 */
    if (now_ms() < s_cooldown_until) {
        return;
    }

    /* 5) 规则表 */
    if (s_last.sick_ratio >= SICK_SEVERE) {
        act_sick_severe();
        s_cooldown_until = now_ms() + COOLDOWN_MS;
    } else if (s_last.sick_ratio >= SICK_MILD) {
        act_sick_mild();
        s_cooldown_until = now_ms() + COOLDOWN_MS;
    } else if (s_motion_hits >= MOTION_HITS) {
        act_motion();
        s_motion_hits    = 0;
        s_cooldown_until = now_ms() + COOLDOWN_MS;
    }
}

/* ------------------------------------------------------------------ */
/* 定时任务：处理需要「延时」的动作                                     */
/* ------------------------------------------------------------------ */

static void ai_tick_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int64_t now = now_ms();

        /* 强通风到点关闭 */
        if (s_fan_off_at != 0 && now >= s_fan_off_at) {
            s_fan_off_at = 0;
            uart_send_to_stm32("{\"fan\":0}");
            log_task_post_event("AI_FAN_OFF");
            ESP_LOGI(TAG, "通风结束，风扇关闭");
        }

        /* 空箱持续 → 关补光省电 */
        if (s_empty_since != 0 && !s_empty_done &&
            (now - s_empty_since) > EMPTY_MS) {
            s_empty_done = true;
            uart_send_to_stm32("{\"light_level\":0}");
            log_task_post_event("AI_EMPTY_LIGHT_OFF");
            snprintf(s_last_action, sizeof(s_last_action), "AI_EMPTY_LIGHT_OFF");
            s_actions++;
            ESP_LOGI(TAG, "空箱超过 %d 秒，补光关闭", EMPTY_MS / 1000);
        }

        /* 视觉离线提示 */
        if (s_last_us != 0 && !s_stale_warned &&
            (esp_timer_get_time() - s_last_us) > (int64_t)STALE_MS * 1000) {
            s_stale_warned = true;
            log_task_post_event("AI_VISION_LOST");
            ESP_LOGW(TAG, "超过 %d 秒没有新的推理结果（PC 侧可能掉线）",
                     STALE_MS / 1000);
        }
    }
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */

void ai_policy_init(void)
{
    memset(&s_last, 0, sizeof(s_last));
    s_last.conf = -1.0f;
    xTaskCreatePinnedToCore(ai_tick_task, "ai_tick", 3072, NULL, 2, NULL, 0);
    ESP_LOGI(TAG, "AI 决策模块就绪（置信门槛 %.2f，病蚕 %.0f%%/%.0f%%，冷却 %d 秒）",
             (double)CONF_MIN, (double)(SICK_MILD * 100), (double)(SICK_SEVERE * 100),
             COOLDOWN_MS / 1000);
}

void ai_policy_get(ai_result_t *out, int *age_ms, const char *last_action,
                   size_t last_action_len, int *actions, bool *enabled)
{
    if (out) {
        *out = s_last;
    }
    if (age_ms) {
        *age_ms = (s_last_us == 0) ? -1 : (int)((esp_timer_get_time() - s_last_us) / 1000);
    }
    if (last_action) {
        snprintf((char *)last_action, last_action_len, "%s", s_last_action);
    }
    if (actions) {
        *actions = s_actions;
    }
    if (enabled) {
        *enabled = s_enabled;
    }
}

void ai_policy_set_enabled(bool on)
{
    s_enabled = on;
    ESP_LOGI(TAG, "AI 决策 %s", on ? "开启" : "关闭");
}
