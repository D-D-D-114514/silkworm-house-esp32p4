/*
 * 摄像头任务 —— 官方 esp_video 路线（V4L2）
 *
 * 之前手写 esp_cam_ctlr + CSI 一直收不到帧，改用乐鑫官方的 esp_video 组件：
 *   OV5647 --MIPI CSI--> ISP(去马赛克/自动曝光) --+--> RGB565
 *                                                +--> 硬件 JPEG 直出
 *
 * esp_video 内部已经处理好：MIPI LDO 2.5V 供电、传感器的 IPA 参数
 * （Bayer 顺序、AWB/AE）、lane 码率、ISP pipeline 控制器。
 * 应用层只需要按 V4L2 标准流程 open/ioctl/mmap/DQBUF。
 *
 * 输出优先选"设备直出 JPEG"，若该格式不支持则退化为 RGB565 + 自己编码。
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "camera_task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_private/esp_cache_private.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"
#include "linux/videodev2.h"
#include "driver/i2c_master.h"
#include "driver/jpeg_encode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "CAMERA";

#define CAM_SCCB_SDA_GPIO   7
#define CAM_SCCB_SCL_GPIO   8
#define CAM_I2C_PORT        0
#define CAM_SCCB_FREQ       100000

#define CAM_W               1920
#define CAM_H               1080
#define BUF_COUNT           2

#define JPEG_BUF_SIZE       (2048 * 1024)   /* 1080p 高噪点画面 q92 可能到 1MB 以上 */
#define JPEG_QUALITY        80
#define DENOISE_LEVEL       12      /* 硬件 Bayer 降噪强度 2~20 */
#define FRAME_DROP_EVERY    1       /* 1=每帧都出；2=隔一帧丢一帧（30→15fps） */
#define AE_TARGET_LUMA      110     /* 软件测光的目标亮度（0~255，G 通道） */

static int  s_fd = -1;
static bool s_dev_jpeg = false;          /* 设备是否直出 JPEG */
static jpeg_encoder_handle_t s_jpeg = NULL;
static uint32_t s_pixfmt = 0;            /* 实际协商到的像素格式（4CC） */

static uint8_t *s_mmap[BUF_COUNT];
static size_t   s_mmap_len[BUF_COUNT];

static uint8_t *s_out_buf[2] = { NULL, NULL };
static size_t   s_out_len[2] = { 0, 0 };
static volatile int s_out_pub = -1;
static volatile uint32_t s_frame_seq = 0;   /* 每发布一帧 +1，供录制判断新帧 */

/* 发送侧快照：HTTP 任务拿到的是这一份拷贝，不是采集任务正在写的缓冲。
 * 否则一边发 400KB（约 40ms）一边被新帧覆盖，JPEG 尾部被截断，
 * 浏览器解码出来就是"底部一条亮绿横带"。 */
static uint8_t *s_snap_buf = NULL;
static SemaphoreHandle_t s_snap_lock = NULL;

/* ---------------- OV5647 寄存器直写 ----------------
 * 乐鑫的驱动完全没有实现曝光/增益设置，传感器内部 AEC/AGC 一直在自己跑 ——
 * 这是噪点大、补光就过曝、曝光来回闪的根源。
 * 这里自己持有 I2C 总线句柄，把控制权拿回来。
 * 寄存器定义参考 Linux 内核 drivers/media/i2c/ov5647.c
 */
#define OV5647_REG_EXP_HI       0x3500
#define OV5647_REG_EXP_MID      0x3501
#define OV5647_REG_EXP_LO       0x3502
#define OV5647_REG_AEC_AGC      0x3503
#define OV5647_REG_GAIN_HI      0x350A
#define OV5647_REG_GAIN_LO      0x350B
#define OV5647_REG_VTS_HI       0x380E
#define OV5647_REG_VTS_LO       0x380F

#define OV5647_ADDR             0x36
#define EXP_MIN_LINES           4           /* 最短 4 行 ≈ 0.11ms，强补光下也压得住 */
#define GAIN_1X                 16          /* 10 位增益，16 = 1.0x，最高 1023 ≈ 4x */
#define GAIN_MAX                64          /* 自动测光最多加到 4x，别放它冲到 64x */

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_ov_dev  = NULL;

static volatile uint32_t s_exp_lines = 900;         /* 当前曝光，单位=行 */
static volatile uint16_t s_gain_val  = 32;          /* 当前增益：16=1x，32=2x */
static uint32_t s_max_exp = 1195;                   /* VTS - 4，运行时读寄存器 */
static uint32_t s_vts = 1199;                       /* 当前 VTS（帧时间，单位=行） */

/* 目标帧率对应的 VTS：fps ≈ 36000 / VTS（tline≈27.78µs）
 * VTS 越大帧率越低，但单帧可用的曝光时间越长 → 进光更多、噪声更低。
 *   1199 → 30fps，曝光上限  33ms（原来的设置）
 *   2400 → 15fps，曝光上限  67ms
 *   3000 → 12fps，曝光上限  83ms
 *   3600 → 10fps，曝光上限 100ms   ← 当前选择（100ms = 5 个 50Hz 周期，不闪）
 * 录像按 10fps 采样，相机留到 12fps 保证每次都有新帧。 */
#define CAM_VTS_TARGET  3600
static int      s_test_stage = -1;                  /* 开机响应自检；-1 表示关闭 */
#define TEST_STEPS 11

static volatile bool     s_auto      = false;       /* 默认手动，由网页控制 */
static volatile uint32_t s_target    = AE_TARGET_LUMA;
static volatile uint32_t s_last_mean = 0;
static volatile bool     s_sensor_ready = false;

static esp_err_t ov_write(uint16_t reg, uint8_t val)
{
    if (!s_ov_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    return i2c_master_transmit(s_ov_dev, buf, sizeof(buf), 100);
}

static esp_err_t ov_read(uint16_t reg, uint8_t *val)
{
    if (!s_ov_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(s_ov_dev, a, sizeof(a), val, 1, 100);
}

static void ov_set_exposure(uint32_t lines)
{
    ov_write(OV5647_REG_EXP_HI,  (lines >> 12) & 0x0F);
    ov_write(OV5647_REG_EXP_MID, (lines >> 4)  & 0xFF);
    ov_write(OV5647_REG_EXP_LO,  (lines & 0x0F) << 4);
}

static void ov_set_gain(uint16_t g)
{
    ov_write(OV5647_REG_GAIN_HI, (g >> 8) & 0x03);
    ov_write(OV5647_REG_GAIN_LO,  g       & 0xFF);
}

/* 设置 VTS（一帧的行数）。VTS 越大帧率越低，但曝光上限越高 */
static void ov_set_vts(uint32_t vts)
{
    if (vts < 1120) {
        vts = 1120;         /* 不能小于有效行数，否则画面会断 */
    }
    if (vts > 65535) {
        vts = 65535;
    }
    ov_write(OV5647_REG_VTS_HI, (vts >> 8) & 0xFF);
    ov_write(OV5647_REG_VTS_LO,  vts       & 0xFF);
    s_vts = vts;
    s_max_exp = vts - 4;
    if (s_exp_lines > s_max_exp) {
        s_exp_lines = s_max_exp;
        ov_set_exposure(s_exp_lines);
    }
}

/* 回读：用来验证我们写的值到底有没有留在寄存器里
 * 如果读回来和我们写的不一样，说明有别人（ISP 管线）在同时写 */
static uint32_t ov_get_exposure(void)
{
    uint8_t h = 0, m = 0, l = 0;
    ov_read(OV5647_REG_EXP_HI, &h);
    ov_read(OV5647_REG_EXP_MID, &m);
    ov_read(OV5647_REG_EXP_LO, &l);
    return ((uint32_t)(h & 0x0F) << 12) | ((uint32_t)m << 4) | ((uint32_t)(l >> 4) & 0x0F);
}

static uint16_t ov_get_gain(void)
{
    uint8_t h = 0, l = 0;
    ov_read(OV5647_REG_GAIN_HI, &h);
    ov_read(OV5647_REG_GAIN_LO, &l);
    return ((uint16_t)(h & 0x03) << 8) | l;
}

static uint8_t ov_get_aec(void)
{
    uint8_t v = 0xFF;
    ov_read(OV5647_REG_AEC_AGC, &v);
    return v;
}

/* 接管曝光/增益：0x3503 = 0x03 → bit0 关 AEC、bit1 关 AGC，改为全手动 */
static void cam_take_ae_control(void)
{
    uint8_t hi = 0, lo = 0;
    if (ov_read(OV5647_REG_VTS_HI, &hi) == ESP_OK &&
            ov_read(OV5647_REG_VTS_LO, &lo) == ESP_OK) {
        uint32_t vts = ((uint32_t)hi << 8) | lo;
        if (vts > 100) {
            s_vts = vts;
            s_max_exp = vts - 4;
        }
    }

    esp_err_t r = ov_write(OV5647_REG_AEC_AGC, 0x03);

    /* 降帧率换曝光时间：这是画质的关键一步，不是性能妥协 */
    ov_set_vts(CAM_VTS_TARGET);

    ov_set_gain(s_gain_val);
    ov_set_exposure(s_exp_lines);
    s_sensor_ready = true;

    ESP_LOGI(TAG, "AE/AGC 接管: 写 0x3503=0x03 (%s)", esp_err_to_name(r));
    ESP_LOGI(TAG, "  回读 0x3503=0x%02X  曝光 写%u→读%u  增益 写%u→读%u  VTS=%u 曝光上限=%u",
             ov_get_aec(), (unsigned)s_exp_lines, (unsigned)ov_get_exposure(),
             (unsigned)s_gain_val, (unsigned)ov_get_gain(),
             (unsigned)(s_max_exp + 4), (unsigned)s_max_exp);
    ESP_LOGI(TAG, "帧率设定: VTS=%u → %.1f fps，单帧最长曝光 %.1f ms（30fps 时为 33ms）",
             (unsigned)s_vts, 36000.0 / (double)s_vts,
             (double)s_max_exp * 27.78 / 1000.0);
}

/* 软件测光：用 RGB565 的绿色通道估亮度，只动曝光，增益尽量锁 1x */
static void cam_ae_step(const uint8_t *rgb565, size_t bytes)
{
    if (!s_ov_dev || bytes < 64) {
        return;
    }

    uint32_t sum_le = 0, sum_be = 0, sum_y = 0, n = 0;
    if (s_pixfmt == V4L2_PIX_FMT_YUYV) {
        /* YUYV：每 2 字节一对，偶数字节就是 Y（亮度） */
        for (size_t i = 0; i + 1 < bytes; i += 64) {
            sum_y += rgb565[i];
            n++;
        }
    } else {
        /* RGB565：字节序不确定，两种都算，谁大用谁（宁可欠曝也不要过曝） */
        for (size_t i = 0; i + 1 < bytes; i += 64) {    /* 每 32 像素采一个点 */
            uint8_t a = rgb565[i], b = rgb565[i + 1];
            sum_le += ((b & 0x07) << 3) | (a >> 5);     /* 假定小端 */
            sum_be += ((a & 0x07) << 3) | (b >> 5);     /* 假定大端 */
            n++;
        }
    }
    if (n == 0) {
        return;
    }

    uint32_t g_le = sum_le / n;
    uint32_t g_be = sum_be / n;
    uint32_t mean = (s_pixfmt == V4L2_PIX_FMT_YUYV) ? (sum_y / n)
                                                    : (g_le > g_be ? g_le : g_be);
    s_last_mean = mean;

    /* 手动模式：只记录亮度，不动曝光和增益 */
    if (!s_auto) {
        return;
    }

    /* ---- 开机响应自检：先扫曝光，再扫增益，打出响应曲线 ---- */
    if (s_test_stage >= 0 && s_test_stage <= TEST_STEPS) {
        static const uint32_t te[5] = { 100, 300, 600, 900, 1195 };
        static const uint16_t tg[6] = { 16, 32, 64, 128, 512, 1023 };

        if (s_test_stage < TEST_STEPS) {
            ESP_LOGW(TAG, "[自检%d] 曝光=%u行 增益=%u(%.2fx) → 实测亮度=%u (LE=%u BE=%u)",
                     s_test_stage, (unsigned)s_exp_lines, (unsigned)s_gain_val,
                     s_gain_val / 16.0f, (unsigned)mean, (unsigned)g_le, (unsigned)g_be);
        }

        s_test_stage++;
        if (s_test_stage < 5) {
            s_exp_lines = te[s_test_stage];
            s_gain_val  = 32;
        } else if (s_test_stage < TEST_STEPS) {
            s_exp_lines = 1195;
            s_gain_val  = tg[s_test_stage - 5];
        } else {
            s_exp_lines = 700;
            s_gain_val  = GAIN_1X;
            ESP_LOGW(TAG, "[自检] 完成，恢复 曝光=%u 增益=%u",
                     (unsigned)s_exp_lines, (unsigned)s_gain_val);
        }
        ov_set_exposure(s_exp_lines);
        ov_set_gain(s_gain_val);
        return;
    }

    /* 每 2 秒打印一次状态（含寄存器回读）—— 无论是否在死区都要打 */
    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 2000000) {
        last_log = now;
        ESP_LOGI(TAG, "AE: 用%u (G_LE=%u G_BE=%u) 曝光=%u(读回%u) 增益=%.2fx(读回%u) 0x3503=0x%02X",
                 (unsigned)mean, (unsigned)g_le, (unsigned)g_be,
                 (unsigned)s_exp_lines, (unsigned)ov_get_exposure(),
                 s_gain_val / 16.0f, (unsigned)ov_get_gain(), ov_get_aec());
    }

    /* 硬保护：极端过曝/欠曝立刻减半/翻倍，不走比例步进 */
    if (mean > 230 || mean < 12) {
        uint32_t e = (mean > 230) ? (s_exp_lines / 2) : (s_exp_lines * 2);
        if (e < EXP_MIN_LINES) {
            e = EXP_MIN_LINES;
        }
        if (e > s_max_exp) {
            e = s_max_exp;
        }
        if (e != s_exp_lines) {
            s_exp_lines = e;
            ov_set_exposure(e);
            ESP_LOGW(TAG, "极端亮度 %u → 曝光直接改为 %u 行", (unsigned)mean, (unsigned)e);
        }
        return;
    }

    int err = (int)s_target - (int)mean;
    if (err >= -4 && err <= 4) {
        return;                                         /* 死区，避免来回抖 */
    }

    uint32_t exp = s_exp_lines;
    if (err > 0) {
        uint32_t step = exp * (uint32_t)err / 220 + 1;
        exp = (exp + step > s_max_exp) ? s_max_exp : exp + step;
    } else {
        uint32_t step = exp * (uint32_t)(-err) / 160 + 1;
        exp = (exp > step + EXP_MIN_LINES) ? exp - step : EXP_MIN_LINES;
    }

    if (exp != s_exp_lines) {
        s_exp_lines = exp;
        ov_set_exposure(exp);
    }

    /* 曝光顶满还偏暗，才动增益；上限卡在 GAIN_MAX(4x)，不许冲到 64x 噪点炸弹 */
    if (exp >= s_max_exp && err > 6 && s_gain_val < GAIN_MAX) {
        uint16_t g = s_gain_val + 8;
        s_gain_val = (g > GAIN_MAX) ? GAIN_MAX : g;
        ov_set_gain(s_gain_val);
    } else if (s_gain_val > GAIN_1X && exp < s_max_exp / 2) {
        s_gain_val = GAIN_1X;
        ov_set_gain(s_gain_val);
    }

}

static esp_err_t cam_video_init(void)
{
    /* 自己建 I2C 总线并持有句柄，才能直接写传感器寄存器 */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CAM_SCCB_SDA_GPIO,
        .scl_io_num = CAM_SCCB_SCL_GPIO,
        .i2c_port   = CAM_I2C_PORT,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线创建失败");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OV5647_ADDR,
        .scl_speed_hz    = CAM_SCCB_FREQ,
    };
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_ov_dev) != ESP_OK) {
        ESP_LOGE(TAG, "OV5647 设备句柄创建失败");
        return ESP_FAIL;
    }

    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb  = false,        /* 用我们自己的总线 */
            .i2c_handle = s_i2c_bus,
            .freq = CAM_SCCB_FREQ,
        },
        .reset_pin = -1,
        .pwdn_pin  = -1,
    };

    esp_video_init_config_t vcfg = {
        .csi = &csi_cfg,
    };

    ESP_LOGI(TAG, "[1/4] esp_video 初始化（含 MIPI LDO / ISP / 传感器探测）...");
    esp_err_t ret = esp_video_init(&vcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static esp_err_t cam_open_video(void)
{
    s_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (s_fd < 0) {
        ESP_LOGE(TAG, "打开 %s 失败", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return ESP_FAIL;
    }

    struct v4l2_format fmt = { 0 };
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_W;
    fmt.fmt.pix.height      = CAM_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;

    if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) == 0) {
        s_dev_jpeg = true;
        s_pixfmt = fmt.fmt.pix.pixelformat;
        ESP_LOGI(TAG, "[2/4] 格式: JPEG 直出 %ux%u（设备端编码，无需软件处理）",
                 (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height);
    } else {
        /* 退化：ISP 输出 RGB565，我们自己用硬件 JPEG 编码器压 */
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = CAM_W;
        fmt.fmt.pix.height      = CAM_H;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) {
            ESP_LOGE(TAG, "JPEG 和 RGB565 都设置失败，设备可能没识别到传感器");
            close(s_fd);
            s_fd = -1;
            return ESP_FAIL;
        }
        s_dev_jpeg = false;
        s_pixfmt = fmt.fmt.pix.pixelformat;

        /* 必须回读协商结果：驱动可能静默改格式（RGB565 → YUYV 之类），
         * 一旦不是 RGB565，后面测光和编码都得跟着变 */
        const uint8_t *p = (const uint8_t *)&s_pixfmt;
        ESP_LOGW(TAG, "[2/4] 协商格式: '%c%c%c%c' %ux%u  bytesperline=%u sizeimage=%u",
                 p[0], p[1], p[2], p[3],
                 (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                 (unsigned)fmt.fmt.pix.bytesperline, (unsigned)fmt.fmt.pix.sizeimage);

        if (s_pixfmt != V4L2_PIX_FMT_RGB565 && s_pixfmt != V4L2_PIX_FMT_YUYV) {
            ESP_LOGE(TAG, "不支持的格式，停止");
            close(s_fd);
            s_fd = -1;
            return ESP_FAIL;
        }
    }

    struct v4l2_requestbuffers req = { 0 };
    req.count  = BUF_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(s_fd, VIDIOC_REQBUFS, &req) != 0 || req.count < BUF_COUNT) {
        ESP_LOGE(TAG, "申请视频缓冲失败");
        return ESP_FAIL;
    }

    for (int i = 0; i < BUF_COUNT; i++) {
        struct v4l2_buffer buf = { 0 };
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(s_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QUERYBUF %d 失败", i);
            return ESP_FAIL;
        }
        s_mmap_len[i] = buf.length;
        s_mmap[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_fd, buf.m.offset);
        if (s_mmap[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap %d 失败", i);
            return ESP_FAIL;
        }
        if (ioctl(s_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF %d 失败", i);
            return ESP_FAIL;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON 失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "[3/4] 视频流已启动（%d 个 mmap 缓冲）", BUF_COUNT);
    return ESP_OK;
}

/* 硬件 Bayer 降噪（BF）：ESP32-P4 ISP 自带，强度 2~20，越大越平滑 */
static void cam_set_denoise(uint8_t level)
{
    int fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        ESP_LOGW(TAG, "打不开 ISP 设备 %s，跳过降噪配置", ESP_VIDEO_ISP1_DEVICE_NAME);
        return;
    }

    esp_video_isp_bf_t bf = {
        .enable = true,
        .level  = level,
        .matrix = { { 1, 2, 1 }, { 2, 4, 2 }, { 1, 2, 1 } },
    };

    struct v4l2_ext_control ctrl = { 0 };
    ctrl.id   = V4L2_CID_USER_ESP_ISP_BF;
    ctrl.size = sizeof(bf);
    ctrl.p_u8 = (uint8_t *)&bf;

    struct v4l2_ext_controls controls = { 0 };
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count      = 1;
    controls.controls   = &ctrl;

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == 0) {
        ESP_LOGI(TAG, "硬件降噪(BF)已开启，强度=%u/20", (unsigned)level);
    } else {
        ESP_LOGW(TAG, "BF 设置失败 errno=%d", errno);
    }
    close(fd);
}

/* 读当前 AE 自动曝光值，用于判断噪点来源（曝光不足 → AE 拉高增益 → 噪点） */
static void cam_log_exposure(void)
{
    struct v4l2_control c = { .id = V4L2_CID_EXPOSURE };
    if (ioctl(s_fd, VIDIOC_G_CTRL, &c) == 0) {
        ESP_LOGI(TAG, "AE 曝光=%d（范围 2~235，越小说明越亮、增益越低）", c.value);
    }
}

static void cam_capture_task(void *arg)
{
    jpeg_encode_cfg_t enc_cfg = {
        .height        = CAM_H,
        .width         = CAM_W,
        .src_type      = (s_pixfmt == V4L2_PIX_FMT_YUYV) ? JPEG_ENCODE_IN_FORMAT_YUV422
                                                         : JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample    = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = JPEG_QUALITY,
        .pixel_reverse = false,
    };

    ESP_LOGI(TAG, "编码输入: %s",
             s_pixfmt == V4L2_PIX_FMT_YUYV ? "YUV422(YUYV)" : "RGB565");

    int idx = 0, frames = 0;

    while (1) {
        struct v4l2_buffer buf = { 0 };
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(s_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGW(TAG, "DQBUF 失败");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 需要降帧时把 FRAME_DROP_EVERY 设为 2（30→15fps）；
         * 丢帧只影响输出，不影响 AE 和传感器。 */
        static uint32_t cam_seq = 0;
        if (FRAME_DROP_EVERY > 1 && (cam_seq++ % FRAME_DROP_EVERY) == 0) {
            ioctl(s_fd, VIDIOC_QBUF, &buf);
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        uint32_t len = buf.m.planes ? buf.m.planes[0].bytesused : buf.bytesused;
        uint8_t *src = s_mmap[buf.index];

        /* 每 6 帧（约 200ms）测光一次，只调曝光 */
        static uint32_t ae_tick = 0;
        if ((ae_tick++ % 6) == 0) {
            cam_ae_step(src, (size_t)CAM_W * CAM_H * 2);
        }

        esp_err_t ret = ESP_FAIL;
        size_t out_len = 0;

        /* 编码 + 发布期间持锁：读者拿到的永远是完整的一帧 */
        if (s_snap_lock) {
            xSemaphoreTake(s_snap_lock, portMAX_DELAY);
        }

        if (s_dev_jpeg) {
            if (len > 0 && len <= JPEG_BUF_SIZE) {
                memcpy(s_out_buf[idx], src, len);
                out_len = len;
                ret = ESP_OK;
            }
        } else if (s_jpeg) {
            uint32_t enc_size = 0;
            /* 精确帧尺寸：800*800*2 字节（RGB565），不要用 mmap 的总长度 */
            if (jpeg_encoder_process(s_jpeg, &enc_cfg, src, CAM_W * CAM_H * 2,
                                     s_out_buf[idx], JPEG_BUF_SIZE, &enc_size) == ESP_OK) {
                out_len = enc_size;
                ret = ESP_OK;
            }
        }

        ioctl(s_fd, VIDIOC_QBUF, &buf);      /* 立刻还回缓冲，不等编码完 */

        if (ret == ESP_OK && out_len > 0) {
            s_out_len[idx] = out_len;
            s_out_pub = idx;
            s_frame_seq++;
            idx ^= 1;
            frames++;
            if (frames <= 5 || frames % 50 == 0) {
                int64_t dt = esp_timer_get_time() - t0;
                ESP_LOGI(TAG, "第 %d 帧: JPEG %u 字节, 处理耗时 %lld ms",
                         frames, (unsigned)out_len, (long long)(dt / 1000));
            }
            if (frames % 100 == 0) {
                cam_log_exposure();
            }
        } else if (frames == 0) {
            ESP_LOGW(TAG, "首帧处理失败 (bytesused=%u, JPEG 缓冲 %u 字节)",
                     (unsigned)len, (unsigned)JPEG_BUF_SIZE);
        }

        if (s_snap_lock) {
            xSemaphoreGive(s_snap_lock);
        }
    }
}

void camera_task_init(void)
{
    ESP_LOGI(TAG, "===== 摄像头初始化（esp_video / V4L2）=====");

    /* esp_video 的 ISP 会把自身标签强行设成 DEBUG（每帧打印直方图/AE 统计），
     * 115200 波特率扛不住会堵住日志，这里压回 WARN。 */
    esp_log_level_set("ISP", ESP_LOG_WARN);
    esp_log_level_set("csi_video", ESP_LOG_WARN);

    if (cam_video_init() != ESP_OK) {
        return;
    }
    esp_log_level_set("ISP", ESP_LOG_WARN);   /* esp_video_init 内部会重新设，盖一次 */

    size_t align = 0;
    esp_cache_get_alignment(0, &align);
    for (int i = 0; i < 2; i++) {
        s_out_buf[i] = heap_caps_aligned_calloc(align, 1, JPEG_BUF_SIZE,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_out_buf[i]) {
            ESP_LOGE(TAG, "输出缓冲分配失败");
            return;
        }
    }

    s_snap_buf = heap_caps_aligned_calloc(align, 1, JPEG_BUF_SIZE,
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_snap_lock = xSemaphoreCreateMutex();
    if (!s_snap_buf || !s_snap_lock) {
        ESP_LOGE(TAG, "快照缓冲/锁创建失败");
        return;
    }

    if (cam_open_video() != ESP_OK) {
        ESP_LOGE(TAG, "视频设备打开失败");
        return;
    }

    cam_set_denoise(DENOISE_LEVEL);

    /* 夺回曝光/增益控制权（必须在流启动、模式固定之后） */
    cam_take_ae_control();

    if (!s_dev_jpeg) {
        jpeg_encode_engine_cfg_t eng_cfg = {
            .intr_priority = 0,
            .timeout_ms = 2000,
            .flags.allow_pd = 0,
        };
        if (jpeg_new_encoder_engine(&eng_cfg, &s_jpeg) != ESP_OK) {
            ESP_LOGE(TAG, "JPEG 编码器创建失败");
            return;
        }
    }

    xTaskCreatePinnedToCore(cam_capture_task, "cam_capture", 8192, NULL, 6, NULL, 0);
    ESP_LOGI(TAG, "[4/4] 采集任务已启动");
}

esp_err_t camera_get_latest_frame(uint8_t **out_buf, size_t *out_len, uint32_t *out_seq)
{
    if (!s_snap_lock || !s_snap_buf) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_snap_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    int pub = s_out_pub;
    if (pub >= 0 && s_out_len[pub] > 0) {
        memcpy(s_snap_buf, s_out_buf[pub], s_out_len[pub]);
        *out_buf = s_snap_buf;
        *out_len = s_out_len[pub];
        if (out_seq) {
            *out_seq = s_frame_seq;
        }
        ret = ESP_OK;
    }

    xSemaphoreGive(s_snap_lock);
    return ret;
}


esp_err_t camera_copy_latest_frame(uint8_t *dst, size_t cap, size_t *out_len,
                                   uint32_t *out_seq)
{
    if (!s_snap_lock || dst == NULL || cap == 0) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_snap_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    int pub = s_out_pub;
    if (pub >= 0 && s_out_len[pub] > 0 && s_out_len[pub] <= cap) {
        memcpy(dst, s_out_buf[pub], s_out_len[pub]);
        if (out_len) {
            *out_len = s_out_len[pub];
        }
        if (out_seq) {
            *out_seq = s_frame_seq;
        }
        ret = ESP_OK;
    }

    xSemaphoreGive(s_snap_lock);
    return ret;
}

/* ---------------- 网页手动调参接口 ---------------- */

void camera_set_exposure(uint32_t lines)
{
    if (!s_sensor_ready) {
        return;
    }
    if (lines < EXP_MIN_LINES) {
        lines = EXP_MIN_LINES;
    }
    if (lines > s_max_exp) {
        lines = s_max_exp;
    }
    s_exp_lines = lines;
    ov_set_exposure(lines);
}

void camera_set_gain(uint16_t gain)
{
    if (!s_sensor_ready) {
        return;
    }
    if (gain < GAIN_1X) {
        gain = GAIN_1X;
    }
    if (gain > 1023) {
        gain = 1023;
    }
    s_gain_val = gain;
    ov_set_gain(gain);
}

void camera_set_auto(bool on)   { s_auto = on; }
void camera_set_target(uint32_t t)
{
    if (t < 20)  t = 20;
    if (t > 240) t = 240;
    s_target = t;
}

uint32_t camera_get_exposure(void)     { return s_exp_lines; }
uint16_t camera_get_gain(void)         { return s_gain_val; }
uint32_t camera_get_max_exposure(void) { return s_max_exp; }
uint32_t camera_get_target(void)       { return s_target; }
uint32_t camera_get_last_mean(void)    { return s_last_mean; }
bool     camera_get_auto(void)         { return s_auto; }

void camera_set_vts(uint32_t vts)      { if (s_sensor_ready) ov_set_vts(vts); }
uint32_t camera_get_vts(void)          { return s_vts; }

/* 帧率：tline = 2271 / 81.667MHz ≈ 27.78µs，所以 fps ≈ 36000 / VTS */
uint32_t camera_get_fps(void)          { return s_vts ? (36000u / s_vts) : 0; }
