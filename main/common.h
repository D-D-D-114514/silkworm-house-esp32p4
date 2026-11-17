#ifndef COMMON_H
#define COMMON_H

/**
 * @brief 全局共享变量：最新温湿度数据
 * 由 uart_comm_task 更新，log_task 读取
 */
extern float latest_temp;
extern float latest_humidity;

#endif // COMMON_H
