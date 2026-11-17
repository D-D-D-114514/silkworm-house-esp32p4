#ifndef UART_COMM_TASK_H
#define UART_COMM_TASK_H

#include <stdbool.h>
#include <stdint.h>

/**
 * STM32 遥测快照 —— 字段与交接文档 §5.6 一一对应。
 * 线协议：一行一条 JSON，'\n' 结尾，115200 8N1，无 CRC。
 */
typedef struct {
    bool     link_ok;             /* 3.5 秒内收到过遥测 = 在线 */
    uint32_t age_ms;              /* 最近一帧遥测距今毫秒数 */
    uint32_t rx_frames;           /* 累计收到帧数 */
    uint32_t bad_frames;          /* 解析失败帧数 */
    uint32_t half_frames;         /* 半帧超时丢弃次数 */

    uint32_t ms, seq;
    float    tin, hin, tout, hout;
    float    ah_in, ah_out, td_in, td_out, ah_set;
    float    t_set, rh_set;
    int      in_valid, out_valid;
    char     mode[12];
    int      cool, cool_duty, heat, heat_duty;
    int      hum, light, light_level, fan, tec_fan, fault;
    char     fw[24];
} stm_state_t;

/**
 * @brief 初始化 UART 通信任务（含心跳与半帧超时）
 */
void uart_comm_task_init(void);

/**
 * @brief 发送 JSON 指令到 STM32（自动补 '\n'）
 * @param json_str JSON 字符串
 */
void uart_send_to_stm32(const char *json_str);

/**
 * @brief 取一份遥测快照
 */
void stm_get_state(stm_state_t *out);

/** 请求一次 UART 内部回环自测（不需要接线，结果打在日志里） */
void uart_comm_request_loopback(void);

/** 取最近收到的原始行（换行分隔），返回写入字节数。诊断用。 */
int uart_comm_get_raw(char *out, int max);

/** 累计收到的原始行数 */
uint32_t uart_comm_raw_total(void);

/** 上次回环自测结果：0=未测 1=成功 2=失败 */
int uart_comm_loop_result(void);

#endif // UART_COMM_TASK_H
