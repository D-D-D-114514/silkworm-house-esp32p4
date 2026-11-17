/*
 * HTTP 服务器
 *
 *    /        状态页（立即返回，绝不阻塞）
 *    /status  纯文本状态，方便快速自检
 *    /stream  MJPEG 视频流（摄像头就绪后才有数据；无帧时 3 秒后返回 503）
 *
 * 注意：ESP-IDF 的 httpd 是单任务串行处理请求的。
 * 任何 handler 一旦长时间不返回，整个服务器都会被卡住，
 * 所以 stream_handler 必须先探测有没有帧，没有就尽快返回。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "http_server_task.h"
#include "camera_task.h"
#include "uart_comm_task.h"
#include "ai_policy.h"
#include "video_record_task.h"
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "HTTP_SERVER";

#define MJPEG_BOUNDARY "123456789000000000000987654321"

/* ---------------- 视频流 ----------------
 * 关键：httpd 是单任务串行处理请求的。一个长期不返回的 /stream 会把
 * /cam、/ 全部饿死（网页"点了没反应"就是这个原因）。
 * 这里用 httpd_req_async_handler_begin 把 socket 交给独立推送任务，
 * handler 立刻返回，控制台才能同时刷新状态。
 */
static QueueHandle_t s_stream_q = NULL;

static void stream_task(void *arg)
{
    httpd_req_t *req = NULL;

    while (1) {
        if (xQueueReceive(s_stream_q, &req, portMAX_DELAY) != pdTRUE || req == NULL) {
            continue;
        }
        ESP_LOGI(TAG, "客户端连接 /stream");

        uint8_t *frame_buf = NULL;
        size_t frame_len = 0;
        uint32_t seq = 0;
        bool got = false;

        for (int i = 0; i < 30; i++) {
            if (camera_get_latest_frame(&frame_buf, &frame_len, &seq) == ESP_OK) {
                got = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!got) {
            ESP_LOGW(TAG, "无可用视频帧，返回 503");
            httpd_resp_set_status(req, "503 Service Unavailable");
            httpd_resp_set_type(req, "text/plain; charset=utf-8");
            httpd_resp_sendstr(req, "camera: no frame");
            httpd_req_async_handler_complete(req);
            req = NULL;
            continue;
        }

        httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY);
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

        /* 只发"新"帧：相机 10fps，若按高频轮询且不去重，同一帧会被重复发好几次。
         * 去重后：网页帧 == 相机帧 == 录像帧，三者逐帧一一对应。 */
        uint32_t sent_seq = 0;
        bool     have_seq = false;

        while (frame_buf != NULL) {
            if (have_seq && seq == sent_seq) {
                vTaskDelay(pdMS_TO_TICKS(20));
                frame_buf = NULL;
                frame_len = 0;
                if (camera_get_latest_frame(&frame_buf, &frame_len, &seq) != ESP_OK) {
                    break;
                }
                continue;
            }
            sent_seq = seq;
            have_seq = true;

            char part_buf[128];
            int hdr_len = snprintf(part_buf, sizeof(part_buf),
                                   "--" MJPEG_BOUNDARY "\r\n"
                                   "Content-Type: image/jpeg\r\n"
                                   "Content-Length: %u\r\n\r\n",
                                   (unsigned)frame_len);

            if (httpd_resp_send_chunk(req, part_buf, hdr_len) != ESP_OK ||
                    httpd_resp_send_chunk(req, (const char *)frame_buf, frame_len) != ESP_OK ||
                    httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                ESP_LOGW(TAG, "客户端断开");
                break;
            }

            /* 20ms 轮询 = 50Hz。相机 10fps 时每次只发新帧，网页流畅度等于传感器帧率 */
            vTaskDelay(pdMS_TO_TICKS(20));

            frame_buf = NULL;
            frame_len = 0;
            if (camera_get_latest_frame(&frame_buf, &frame_len, &seq) != ESP_OK) {
                break;
            }
        }

        httpd_req_async_handler_complete(req);
        req = NULL;
    }
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_req_t *async_req = NULL;

    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        return ESP_FAIL;
    }
    if (xQueueSend(s_stream_q, &async_req, 0) != pdTRUE) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---------------- 前端页面（编译时嵌入 index.html） ---------------- */

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

/* ---------------- 摄像头调参接口 ----------------
 *   /cam?info=1        查询状态并返回 JSON
 *   /cam?exp=700       设置曝光（行）
 *   /cam?gain=32       设置增益（16 = 1.0x）
 *   /cam?auto=1|0      自动测光 / 锁定
 *   /cam?target=110    自动测光目标亮度
 */
static esp_err_t cam_handler(httpd_req_t *req)
{
    char query[128];
    char val[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "exp", val, sizeof(val)) == ESP_OK) {
            camera_set_exposure((uint32_t)atoi(val));
        }
        if (httpd_query_key_value(query, "gain", val, sizeof(val)) == ESP_OK) {
            camera_set_gain((uint16_t)atoi(val));
        }
        if (httpd_query_key_value(query, "auto", val, sizeof(val)) == ESP_OK) {
            camera_set_auto(atoi(val) != 0);
        }
        if (httpd_query_key_value(query, "target", val, sizeof(val)) == ESP_OK) {
            camera_set_target((uint32_t)atoi(val));
        }
        if (httpd_query_key_value(query, "lock", val, sizeof(val)) == ESP_OK) {
            camera_set_auto(0);
        }
        if (httpd_query_key_value(query, "vts", val, sizeof(val)) == ESP_OK) {
            camera_set_vts((uint32_t)atoi(val));
        }
    }

    char json[320];
    int n = snprintf(json, sizeof(json),
                     "{\"exp\":%u,\"max\":%u,\"gain\":%u,\"auto\":%d,\"target\":%u,"
                     "\"mean\":%u,\"vts\":%u,\"fps\":%u,\"heap\":%u,\"up\":%lld}",
                     (unsigned)camera_get_exposure(), (unsigned)camera_get_max_exposure(),
                     (unsigned)camera_get_gain(), camera_get_auto() ? 1 : 0,
                     (unsigned)camera_get_target(), (unsigned)camera_get_last_mean(),
                     (unsigned)camera_get_vts(), (unsigned)camera_get_fps(),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                     (long long)(esp_timer_get_time() / 1000000));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "ip=192.168.1.10\n"
        "netmask=255.255.255.0\n"
        "free_heap=%u\n"
        "uptime_s=%lld\n"
        "camera=placeholder\n"
        "sdcard=disabled\n",
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
        (long long)(esp_timer_get_time() / 1000000));

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, buf, n);
}

/* ---------------- STM32 遥测 / 命令 ----------------
 *   /stm                    读遥测 JSON
 *   /stm?mode=manual        切换模式（auto / manual）
 *   /stm?set_temp=24.5      温度设定
 *   /stm?set_humi=70        湿度设定
 *   /stm?light_level=96     补光亮度
 *   /stm?all_off=1          紧急全关
 *   /stm?ping=1 / diag=1    探活 / 自检
 */
#define RAW_HTTP_MAX 2048

static esp_err_t stm_handler(httpd_req_t *req)
{
    char query[192];
    char val[32];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char cmd[128] = { 0 };

        if (httpd_query_key_value(query, "mode", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"mode\":\"%s\"}", val);
        } else if (httpd_query_key_value(query, "set_temp", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"set_temp\":%s}", val);
        } else if (httpd_query_key_value(query, "set_humi", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"set_humi\":%s}", val);
        } else if (httpd_query_key_value(query, "light_level", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"light_level\":%s}", val);
        } else if (httpd_query_key_value(query, "all_off", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"all_off\":1}");
        } else if (httpd_query_key_value(query, "raw", val, sizeof(val)) == ESP_OK) {
            static char rawbuf[RAW_HTTP_MAX];
            int rn = uart_comm_get_raw(rawbuf, sizeof(rawbuf));
            char head[96];
            int hn = snprintf(head, sizeof(head),
                              "# 累计 %u 行，以下为最近 %d 条原始内容\n",
                              (unsigned)uart_comm_raw_total(), 8);
            httpd_resp_set_type(req, "text/plain; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            httpd_resp_send_chunk(req, head, hn);
            return httpd_resp_send_chunk(req, rawbuf, rn);
        } else if (httpd_query_key_value(query, "loop", val, sizeof(val)) == ESP_OK) {
            uart_comm_request_loopback();
        } else if (httpd_query_key_value(query, "ping", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"ping\":1}");
        } else if (httpd_query_key_value(query, "diag", val, sizeof(val)) == ESP_OK) {
            snprintf(cmd, sizeof(cmd), "{\"diag\":1}");
        }

        if (cmd[0]) {
            uart_send_to_stm32(cmd);
        }
    }

    stm_state_t st;
    stm_get_state(&st);

    char json[768];
    int n = snprintf(json, sizeof(json),
                     "{\"ok\":%d,\"age\":%u,\"rx\":%u,\"bad\":%u,\"half\":%u,\"seq\":%u,"
                     "\"tin\":%.2f,\"hin\":%.1f,\"tout\":%.2f,\"hout\":%.1f,"
                     "\"ah_in\":%.2f,\"ah_set\":%.2f,\"td_in\":%.2f,\"td_out\":%.2f,"
                     "\"tset\":%.1f,\"rhset\":%.1f,\"in_v\":%d,\"out_v\":%d,\"mode\":\"%s\","
                     "\"cool\":%d,\"cool_d\":%d,\"heat\":%d,\"heat_d\":%d,"
                     "\"hum\":%d,\"light\":%d,\"llevel\":%d,\"fan\":%d,\"tfan\":%d,"
                     "\"fault\":%d,\"loop\":%d}",
                     st.link_ok ? 1 : 0, (unsigned)st.age_ms, (unsigned)st.rx_frames,
                     (unsigned)st.bad_frames, (unsigned)st.half_frames, (unsigned)st.seq,
                     (double)st.tin, (double)st.hin, (double)st.tout, (double)st.hout,
                     (double)st.ah_in, (double)st.ah_set, (double)st.td_in, (double)st.td_out,
                     (double)st.t_set, (double)st.rh_set, st.in_valid, st.out_valid, st.mode,
                     st.cool, st.cool_duty, st.heat, st.heat_duty,
                     st.hum, st.light, st.light_level, st.fan, st.tec_fan, st.fault,
                     uart_comm_loop_result());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

/* ---------------- 初始化 ---------------- */


/* ================= /ai  PC 侧推理结果 =================
 *
 * POST /ai  Content-Type: application/json
 *   {"count":42,"sick":2,"motion":0.12,"conf":0.82}
 *
 * 或 GET（浏览器里手动测试）
 *   /ai?count=42&sick=2&motion=0.12&conf=0.82
 *
 * P4 只负责「按规则表决定动作」，动作通过 UART 下发给 STM32。
 * ====================================================== */

static bool jget_num(const char *json, const char *key, double *out)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (p == NULL) {
        return false;
    }
    p = strchr(p + strlen(pat), ':');
    if (p == NULL) {
        return false;
    }
    *out = strtod(p + 1, NULL);
    return true;
}

static float q_float(httpd_req_t *req, const char *key, float def)
{
    char q[256], v[32];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) {
        return def;
    }
    if (httpd_query_key_value(q, key, v, sizeof(v)) != ESP_OK) {
        return def;
    }
    return (float)atof(v);
}

static esp_err_t ai_handler(httpd_req_t *req)
{
    ai_result_t r;
    memset(&r, 0, sizeof(r));
    r.conf = -1.0f;
    bool has_data = false;

    if (req->method == HTTP_POST) {
        static char body[512];
        if (req->content_len <= 0 || req->content_len >= (int)sizeof(body)) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "content too long");
            return ESP_FAIL;
        }
        int got = httpd_req_recv(req, body, req->content_len);
        if (got <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
            return ESP_FAIL;
        }
        body[got] = 0;

        double v;
        if (jget_num(body, "count", &v))   { r.count = (int)v; has_data = true; }
        if (jget_num(body, "sick", &v))    { r.sick = (int)v; has_data = true; }
        if (jget_num(body, "motion", &v))  { r.motion = (float)v; has_data = true; }
        if (jget_num(body, "conf", &v))    { r.conf = (float)v; has_data = true; }
        if (jget_num(body, "sick_ratio", &v)) { r.sick_ratio = (float)v; has_data = true; }
        if (jget_num(body, "ts", &v))      { r.ts = (int64_t)v; }
        ESP_LOGD(TAG, "AI 收到: %s", body);
    } else {
        char q[256];
        char v[16];
        if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
            httpd_query_key_value(q, "enable", v, sizeof(v)) == ESP_OK) {
            /* /ai?enable=0  只切换开关，不当作一次上报 */
            ai_policy_set_enabled(atoi(v) != 0);
        } else if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
            has_data = true;          /* 有查询串就当作一次真实上报 */
            r.count    = (int)q_float(req, "count", 0);
            r.sick     = (int)q_float(req, "sick", 0);
            r.motion   = q_float(req, "motion", 0);
            r.conf     = q_float(req, "conf", -1);
            r.sick_ratio = q_float(req, "ratio", 0);
        }
    }

    /* 不带任何参数访问 /ai 时只读状态，不提交（避免误触发空箱/离线判定） */
    if (has_data) {
        ai_policy_submit(&r);
    }

    ai_result_t last;
    char act[64];
    int  n_act;
    bool en;
    int  age;
    ai_policy_get(&last, &age, act, sizeof(act), &n_act, &en);

    char out[320];
    int n = snprintf(out, sizeof(out),
                     "{\"ok\":1,\"enabled\":%d,\"count\":%d,\"sick\":%d,"
                     "\"ratio\":%.3f,\"motion\":%.2f,\"conf\":%.2f,"
                     "\"last_action\":\"%s\",\"actions\":%d,\"age\":%d}",
                     en ? 1 : 0, last.count, last.sick,
                     (double)last.sick_ratio, (double)last.motion, (double)last.conf,
                     act, n_act, age);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}

/* ================= /time  PC 给板子对时 =================
 *
 *   /time?epoch=1731220000      设置时间
 *   /time                       读取当前时间（顺带回显 ISO8601）
 * ====================================================== */

static esp_err_t time_handler(httpd_req_t *req)
{
    char q[128], v[32];
    long long epoch = 0;

    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "epoch", v, sizeof(v)) == ESP_OK) {
        epoch = atoll(v);
    }

    if (epoch > 1600000000LL) {
        struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        struct tm tm;
        localtime_r((time_t *)&tv.tv_sec, &tm);
        ESP_LOGI(TAG, "PC 对时: %04d-%02d-%02d %02d:%02d:%02d",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min, tm.tm_sec);
    }

    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    char iso[32];
    strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%S", &tm);

    char out[128];
    int n = snprintf(out, sizeof(out),
                     "{\"ok\":1,\"ts\":%lld,\"iso\":\"%s\",\"synced\":%d}",
                     (long long)now, iso, (now > 1600000000) ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out, n);
}


static esp_err_t sd_handler(httpd_req_t *req)
{
    static char buf[2048];
    int n = video_list_files(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

void http_server_task_init(void)
{
    ESP_LOGI(TAG, "Starting HTTP server...");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.stack_size = 8192;

    s_stream_q = xQueueCreate(1, sizeof(httpd_req_t *));
    if (s_stream_q == NULL || xTaskCreate(stream_task, "mjpeg", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "视频推送任务创建失败");
        return;
    }

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL
    };
    httpd_uri_t status_uri = {
        .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL
    };
    httpd_uri_t stream_uri = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL
    };
    httpd_uri_t cam_uri = {
        .uri = "/cam", .method = HTTP_GET, .handler = cam_handler, .user_ctx = NULL
    };
    httpd_uri_t stm_uri = {
        .uri = "/stm", .method = HTTP_GET, .handler = stm_handler, .user_ctx = NULL
    };
    httpd_uri_t ai_uri = {
        .uri = "/ai", .method = HTTP_ANY, .handler = ai_handler, .user_ctx = NULL
    };
    httpd_uri_t time_uri = {
        .uri = "/time", .method = HTTP_GET, .handler = time_handler, .user_ctx = NULL
    };
    httpd_uri_t sd_uri = {
        .uri = "/sd", .method = HTTP_GET, .handler = sd_handler, .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &cam_uri);
    httpd_register_uri_handler(server, &stm_uri);
    httpd_register_uri_handler(server, &ai_uri);
    httpd_register_uri_handler(server, &time_uri);
    httpd_register_uri_handler(server, &sd_uri);

    ESP_LOGI(TAG, "[OK] HTTP server started on port 80");
    ESP_LOGI(TAG, "    http://192.168.1.10/        状态页");
    ESP_LOGI(TAG, "    http://192.168.1.10/status  纯文本状态");
    ESP_LOGI(TAG, "    http://192.168.1.10/stream  视频流");
    ESP_LOGI(TAG, "    http://192.168.1.10/cam?info=1  摄像头调参接口");
    ESP_LOGI(TAG, "    http://192.168.1.10/stm          STM32 遥测/命令");
    ESP_LOGI(TAG, "    http://192.168.1.10/ai           PC 推理结果入口");
    ESP_LOGI(TAG, "    http://192.168.1.10/time?epoch=   PC 对时");
    ESP_LOGI(TAG, "    http://192.168.1.10/sd           录像文件列表");
}
