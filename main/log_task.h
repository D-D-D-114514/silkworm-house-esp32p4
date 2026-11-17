#ifndef LOG_TASK_H
#define LOG_TASK_H

/**
 * @brief 初始化日志记录任务
 * 定期将温湿度和系统状态写入 SD 卡
 */
void log_task_init(void);

/**
 * @brief 由其它任务写入一条事件（非阻塞，队列满则丢弃）
 * 用于 AI 决策、视觉事件等，最终落到当天的 CSV 里（kind=E）
 */
void log_task_post_event(const char *name);

#endif // LOG_TASK_H
