/*
 * ESP32-P4 智能蚕房监测系统 - 主程序
 * 功能：摄像头采集、SD卡录制、HTTP视频流、UART通信、日志记录
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include <time.h>

// 任务头文件
#include "camera_task.h"
#include "video_record_task.h"
#include "http_server_task.h"
#include "uart_comm_task.h"
#include "log_task.h"
#include "ai_policy.h"
#include "ethernet_init.h"

static const char *TAG = "MAIN";

/* ------------------------------------------------------------------
 * SNTP 对时
 *
 * 日志的时间戳依赖系统时钟。以太网插得晚（甚至开机几分钟后才插），
 * 所以这里不停重试，最多等 3 分钟。没对上也不影响功能，只是日志的
 * ts 列会显示 1970 年 —— 那时以 ms 列（开机毫秒）为准。
 * ------------------------------------------------------------------ */
static void sntp_sync_task(void *arg)
{
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_init();

    for (int i = 0; i < 90; i++) {           /* 90 x 2s = 180s */
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            time_t now = time(NULL);
            struct tm tm;
            localtime_r(&now, &tm);
            ESP_LOGI("SNTP", "[OK] 对时成功: %04d-%02d-%02d %02d:%02d:%02d",
                     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                     tm.tm_hour, tm.tm_min, tm.tm_sec);
            vTaskDelete(NULL);
        }
    }
    ESP_LOGW("SNTP", "对时失败，日志时间戳会显示 1970 年（ms 列仍然准确）");
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-P4 Smart Silkworm System v1.0  ");
    ESP_LOGI(TAG, "========================================");

    // 初始化 NVS（用于存储关键事件）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "[✓] NVS initialized");

    // 记录启动次数到 NVS
    nvs_handle_t nvs_handle;
    ret = nvs_open("system", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        uint32_t boot_count = 0;
        nvs_get_u32(nvs_handle, "boot_count", &boot_count);
        boot_count++;
        nvs_set_u32(nvs_handle, "boot_count", boot_count);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "[✓] Boot count: %lu", boot_count);
    }

    // 任务1：摄像头初始化与采集
    ESP_LOGI(TAG, "Starting camera task...");
    camera_task_init();

    // 任务2:初始化网络栈（先启动 lwIP TCP/IP 任务）
    ESP_LOGI(TAG, "Initializing network stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 等待 lwIP TCP/IP 任务完全启动（避免 "Invalid mbox" 错误）
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "[✓] Network stack ready");
    
    // 初始化以太网
    ethernet_init();

    // SNTP 对时（日志时间戳依赖它；以太网可能还没插，任务内部会重试 3 分钟）
    xTaskCreatePinnedToCore(sntp_sync_task, "sntp", 4096, NULL, 1, NULL, 0);

    // 任务3：HTTP 视频流服务器
    ESP_LOGI(TAG, "Starting HTTP server task...");
    http_server_task_init();

    // 任务4：SD 卡挂载与视频录制
    ESP_LOGI(TAG, "Starting video recording task...");
    video_record_task_init();

    // 任务5：UART 通信（与 STM32）
    ESP_LOGI(TAG, "Starting UART communication task...");
    uart_comm_task_init();

    // 任务6：AI 决策（PC 送推理结果，这里出动作）
    ai_policy_init();

    // 任务7：日志记录任务
    ESP_LOGI(TAG, "Starting logging task...");
    log_task_init();

    ESP_LOGI(TAG, "[✓] All tasks started successfully");
    ESP_LOGI(TAG, "System is running, press Ctrl+] to exit monitor");

    // 主循环心跳
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 秒心跳
        ESP_LOGI(TAG, "[♥] System alive, heap free: %lu bytes", esp_get_free_heap_size());
    }
}
