#ifndef HTTP_SERVER_TASK_H
#define HTTP_SERVER_TASK_H

/**
 * @brief 启动 HTTP 视频流服务器
 * 端口 80，路径 /stream 提供 MJPEG 流
 */
void http_server_task_init(void);

#endif // HTTP_SERVER_TASK_H
