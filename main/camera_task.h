#ifndef CAMERA_TASK_H
#define CAMERA_TASK_H

#include "esp_err.h"

/**
 * @brief 初始化摄像头任务
 * 配置 OV5647 MIPI-CSI 摄像头并启动采集线程
 */
void camera_task_init(void);

/**
 * @brief 获取最新的一帧 JPEG 数据
 * @param out_buf 输出缓冲区指针
 * @param out_len 输出数据长度
 * @return ESP_OK 成功，ESP_FAIL 失败
 */
esp_err_t camera_get_latest_frame(uint8_t **out_buf, size_t *out_len, uint32_t *out_seq);

/**
 * @brief 把最新一帧 JPEG 拷贝到调用者提供的缓冲区
 *
 * 和 camera_get_latest_frame 的区别：那个返回内部共享缓冲的指针，
 * 释放锁后可能被 /stream 覆盖；录制必须用这个，自己持有副本。
 *
 * @param dst      目标缓冲区
 * @param cap      目标容量
 * @param out_len  实际长度
 * @param out_seq  帧序号（每发布一帧 +1），用于跳过重复帧
 */
esp_err_t camera_copy_latest_frame(uint8_t *dst, size_t cap, size_t *out_len,
                                   uint32_t *out_seq);

/* ---------------- 手动调参接口 ---------------- */

/** 设置曝光（单位：行，1080p 模式上限 1195） */
void camera_set_exposure(uint32_t lines);

/** 设置增益（16 = 1.0x，1023 约 64x） */
void camera_set_gain(uint16_t gain);

/** 开/关自动测光；关 = 锁定当前曝光与增益 */
void camera_set_auto(bool on);

/** 设置自动测光目标亮度（0~255） */
void camera_set_target(uint32_t target);

uint32_t camera_get_exposure(void);
uint16_t camera_get_gain(void);
uint32_t camera_get_max_exposure(void);
uint32_t camera_get_target(void);
uint32_t camera_get_last_mean(void);
bool     camera_get_auto(void);

/** 设置 VTS（一帧的行数）：越大帧率越低、曝光上限越高 */
void     camera_set_vts(uint32_t vts);
uint32_t camera_get_vts(void);
uint32_t camera_get_fps(void);

#endif // CAMERA_TASK_H
