#ifndef AI_POLICY_H
#define AI_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief PC 侧 YOLO11 的推理结果
 *
 * 只传「是什么」，不传「怎么办」—— 决策在 P4。
 */
typedef struct {
    int     count;        /**< 画面里的蚕数 */
    int     sick;         /**< 病蚕数 */
    float   sick_ratio;   /**< 病蚕比例 0..1（为 0 时按 sick/count 推算） */
    float   motion;       /**< 异动指数 0..1 */
    float   conf;         /**< 平均置信度 0..1（<0 表示未提供） */
    int64_t ts;           /**< PC 端时间戳（秒），可填 0 */
} ai_result_t;

/** 启动决策模块（内部起 1s 定时任务，处理风扇超时/空箱/视觉离线） */
void ai_policy_init(void);

/** 提交一次推理结果（由 HTTP 处理函数调用，内部只做判断和下发，不阻塞） */
void ai_policy_submit(const ai_result_t *r);

/** 读取当前状态，供网页展示 */
void ai_policy_get(ai_result_t *out, int *age_ms, const char *last_action,
                   size_t last_action_len, int *actions, bool *enabled);

/** 开 / 关 AI 决策（关掉后只记录不动作） */
void ai_policy_set_enabled(bool on);

#endif // AI_POLICY_H
