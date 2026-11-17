#ifndef VIDEO_RECORD_TASK_H
#define VIDEO_RECORD_TASK_H

#include <stdbool.h>

/**
 * @brief 初始化视频录制任务
 * 挂载 SD 卡，创建录制线程
 */
void video_record_task_init(void);

/** 列出录像文件（JSON），返回写入长度 */
int video_list_files(char *out, int max);

/** SD 卡是否挂载成功（供其它模块判断能否写文件） */
bool sd_card_is_mounted(void);

/** 当前是否正在录制（日志任务据此把采样周期放宽到 10 秒） */
bool video_is_recording(void);

#endif // VIDEO_RECORD_TASK_H
