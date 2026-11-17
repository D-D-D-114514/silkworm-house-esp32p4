/*
 * SD 卡：挂载 + 吞吐自测 + MJPEG(AVI) 分段录制
 *
 * SD 挂载的关键（之前一直 0x107 的原因）：
 *   ESP32-P4 的 MicroSD 供电来自【片内 LDO 通道 4】，必须先
 *   sd_pwr_ctrl_new_on_chip_ldo() 打开，否则卡片压根不上电。
 *   引脚：CLK=43 CMD=44 D0=39 D1=40 D2=41 D3=42（4-bit SDIO）
 *
 * 录像格式：MJPEG 封进标准 AVI（RIFF）
 *   每一帧本身就是一张完整 JPEG，所以：
 *     - VLC / ffmpeg 直接能播
 *     - ffmpeg -i xxx.avi frame_%06d.jpg   就能导出训练集
 *     - 不需要解码整个视频，抽帧极快
 *
 * 帧率：按【实际写入速度】写进文件头，播放速度不失真。
 * 分段：30 分钟（CONFIG_VIDEO_SEGMENT_DURATION_MIN）或 512MB 一段。
 */

#include "video_record_task.h"
#include "camera_task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

static const char *TAG = "VIDEO_REC";

#define MOUNT_POINT   "/sdcard"
#define VIDEO_DIR     MOUNT_POINT "/videos"

#define SD_PIN_CLK 43
#define SD_PIN_CMD 44
#define SD_PIN_D0  39
#define SD_PIN_D1  40
#define SD_PIN_D2  41
#define SD_PIN_D3  42

/* 采样轮询频率：必须比相机帧率快一档，否则会因相位漂移漏帧。
 * 相机 10fps + 轮询 12Hz + 按 seq 去重 → 每一帧都被取到且不重复，
 * 于是 录像帧 == 网页帧，两者逐帧一一对应。 */
#define REC_POLL_HZ    12
#define AVI_WBUF_SIZE  (256 * 1024)  /* stdio 写缓冲：攒够再一次交给 FATFS */
#define SEG_MAX_BYTES  (512u * 1024u * 1024u)
#define KEEP_FILES     64            /* 最多保留多少个分段文件 */

static sdmmc_card_t        *s_card = NULL;
static sd_pwr_ctrl_handle_t s_pwr  = NULL;
static bool                 s_mounted   = false;
static volatile bool        s_recording = false;

/* ================================================================== */
/* AVI (RIFF/MJPG) 写入                                               */
/* ================================================================== */

typedef struct {
    FILE    *fp;
    char     path[80];

    uint32_t w, h;
    uint32_t frames;
    uint32_t max_frame;
    int64_t  t_start_us;

    long off_riff_size;
    long off_avih_frames;
    long off_avih_uspf;
    long off_avih_maxbps;
    long off_avih_flags;
    long off_strh_length;
    long off_strh_scale;
    long off_strh_suggested;
    long off_strh_rate;
    long off_movi_size;
    long movi_fourcc_pos;

    uint32_t *idx;        /* 4 x uint32 per frame */
    uint32_t  idx_cap;
} avi_t;

static void w8(FILE *f, uint8_t v)  { fputc(v, f); }

static void w16(FILE *f, uint16_t v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
}

static void w32(FILE *f, uint32_t v)
{
    fputc(v & 0xFF, f);
    fputc((v >> 8) & 0xFF, f);
    fputc((v >> 16) & 0xFF, f);
    fputc((v >> 24) & 0xFF, f);
}

static void wtag(FILE *f, const char *t) { fwrite(t, 1, 4, f); }

static void patch32(FILE *f, long off, uint32_t v)
{
    long cur = ftell(f);
    fseek(f, off, SEEK_SET);
    w32(f, v);
    fseek(f, cur, SEEK_SET);
}

/* 打开文件并写好 AVI 头（尺寸/帧数先留 0，关闭时回填） */
static bool avi_open(avi_t *a, const char *path, uint32_t w, uint32_t h,
                     uint32_t idx_cap)
{
    memset(a, 0, sizeof(*a));
    a->w = w;
    a->h = h;
    a->idx_cap = idx_cap;
    size_t pl = strlen(path);
    if (pl > sizeof(a->path) - 1) {
        pl = sizeof(a->path) - 1;
    }
    memcpy(a->path, path, pl);
    a->path[pl] = 0;

    a->idx = heap_caps_malloc(idx_cap * 4 * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (a->idx == NULL) {
        ESP_LOGE(TAG, "索引内存分配失败(%u 帧)", (unsigned)idx_cap);
        return false;
    }

    a->fp = fopen(path, "wb");
    if (a->fp == NULL) {
        ESP_LOGE(TAG, "创建 %s 失败 (errno=%d)", path, errno);
        free(a->idx);
        a->idx = NULL;
        return false;
    }

    /* 大缓冲：FATFS 每次 f_write 都有簇分配开销，攒到 256KB 再写能把
     * 实测吞吐从 1.5MB/s 推到 3MB/s 以上（10fps×190KB 需要 1.9MB/s）*/
    static char *vbuf = NULL;
    if (vbuf == NULL) {
        vbuf = heap_caps_malloc(AVI_WBUF_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (vbuf != NULL) {
        setvbuf(a->fp, vbuf, _IOFBF, AVI_WBUF_SIZE);
    }

    FILE *f = a->fp;
    a->t_start_us = esp_timer_get_time();

    wtag(f, "RIFF");
    a->off_riff_size = ftell(f);
    w32(f, 0);
    wtag(f, "AVI ");

    /* ---- LIST hdrl ---- */
    wtag(f, "LIST");
    w32(f, 4 + 64 + 12 + 64 + 48);      /* hdrl 数据长度 */
    wtag(f, "hdrl");

    wtag(f, "avih");
    w32(f, 56);
    a->off_avih_uspf = ftell(f);
    w32(f, 40000);                      /* 先按 25fps，关闭时回填 */
    a->off_avih_maxbps = ftell(f);
    w32(f, 0);                          /* 关闭时回填 */
    w32(f, 0);                          /* padding granularity */
    a->off_avih_flags = ftell(f);
    w32(f, 0);                          /* flags：默认无索引（断电也不会撒谎） */
    a->off_avih_frames = ftell(f);
    w32(f, 0);                          /* dwTotalFrames */
    w32(f, 0);                          /* initial frames */
    w32(f, 1);                          /* streams */
    w32(f, 0);                          /* suggested buffer */
    w32(f, w);
    w32(f, h);
    for (int i = 0; i < 4; i++) {
        w32(f, 0);
    }

    /* ---- LIST strl ---- */
    wtag(f, "LIST");
    w32(f, 4 + 64 + 48);
    wtag(f, "strl");

    wtag(f, "strh");
    w32(f, 56);
    wtag(f, "vids");
    wtag(f, "MJPG");
    w32(f, 0);                          /* flags */
    w16(f, 0);                          /* priority */
    w16(f, 0);                          /* language */
    w32(f, 0);                          /* initial frames */
    a->off_strh_scale = ftell(f);
    w32(f, 1);                          /* dwScale，回填为 us/frame */
    a->off_strh_rate = ftell(f);
    w32(f, 25);                         /* dwRate，回填为 1000000 */
    w32(f, 0);                          /* dwStart */
    a->off_strh_length = ftell(f);
    w32(f, 0);                          /* dwLength */
    a->off_strh_suggested = ftell(f);
    w32(f, 0);                          /* suggested buffer */
    w32(f, 0xFFFFFFFF);                 /* quality */
    w32(f, 0);                          /* sample size */
    w16(f, 0);                          /* rcFrame.left */
    w16(f, 0);                          /* rcFrame.top */
    w16(f, (uint16_t)w);
    w16(f, (uint16_t)h);

    wtag(f, "strf");
    w32(f, 40);
    w32(f, 40);                         /* biSize */
    w32(f, w);
    w32(f, h);
    w16(f, 1);                          /* planes */
    w16(f, 24);                         /* bitcount */
    wtag(f, "MJPG");
    w32(f, w * h * 3);                  /* biSizeImage */
    w32(f, 0);
    w32(f, 0);
    w32(f, 0);
    w32(f, 0);

    /* ---- LIST movi ---- */
    wtag(f, "LIST");
    a->off_movi_size = ftell(f);
    w32(f, 0);                          /* 关闭时回填 */
    a->movi_fourcc_pos = ftell(f);
    wtag(f, "movi");

    return true;
}

static bool avi_add_frame(avi_t *a, const uint8_t *jpg, size_t len)
{
    if (a->fp == NULL || a->idx == NULL) {
        return false;
    }
    if (a->frames >= a->idx_cap) {
        return false;
    }

    long pos = ftell(a->fp);

    wtag(a->fp, "00dc");
    w32(a->fp, (uint32_t)len);
    if (fwrite(jpg, 1, len, a->fp) != len) {
        ESP_LOGE(TAG, "写帧失败 (errno=%d)", errno);
        return false;
    }
    if (len & 1) {
        w8(a->fp, 0);                   /* AVI 块按 2 字节对齐 */
    }

    uint32_t *e = &a->idx[a->frames * 4];
    e[0] = 0x63643030u;                        /* '00dc' */
    e[1] = 0x10u;                              /* AVIIF_KEYFRAME */
    e[2] = (uint32_t)(pos - a->movi_fourcc_pos);
    e[3] = (uint32_t)len;

    if (len > a->max_frame) {
        a->max_frame = (uint32_t)len;
    }
    a->frames++;
    return true;
}


/* 把已写入的数据强制落盘，并把 AVI 头部回填成"到此刻为止有效"
 * 断电后文件仍然可播放（没有 idx1 索引，播放器会顺序扫描） */
static void avi_sync(avi_t *a)
{
    if (a->fp == NULL) {
        return;
    }
    FILE *f = a->fp;

    int64_t elapsed_us = esp_timer_get_time() - a->t_start_us;
    if (elapsed_us <= 0) {
        elapsed_us = 1;
    }
    uint32_t uspf = (uint32_t)(elapsed_us / (a->frames ? a->frames : 1));
    if (uspf < 1000)    uspf = 1000;
    if (uspf > 1000000) uspf = 1000000;
    uint32_t fps = uspf ? (1000000u / uspf) : 1;
    if (fps == 0) {
        fps = 1;
    }

    long pos = ftell(f);

    patch32(f, a->off_riff_size,      (uint32_t)(pos - 8));
    patch32(f, a->off_avih_frames,    a->frames);
    patch32(f, a->off_avih_uspf,      uspf);
    patch32(f, a->off_avih_maxbps,    (uint32_t)(a->max_frame * fps));
    patch32(f, a->off_strh_length,    a->frames);
    patch32(f, a->off_strh_scale,     uspf);        /* dwScale = us/frame */
    patch32(f, a->off_strh_rate,      1000000u);    /* dwRate  = 1e6 */
    patch32(f, a->off_strh_suggested, a->max_frame);
    patch32(f, a->off_movi_size,      (uint32_t)(pos - a->movi_fourcc_pos));

    fflush(f);
    fsync(fileno(f));          /* 关键：逼 FATFS 更新目录项里的文件大小 */
}

static void avi_close(avi_t *a)
{
    if (a->fp == NULL) {
        return;
    }
    FILE *f = a->fp;

    int64_t elapsed_us = esp_timer_get_time() - a->t_start_us;
    if (elapsed_us <= 0) {
        elapsed_us = 1;
    }

    long idx1_pos = ftell(f);
    if (idx1_pos & 1) {
        w8(f, 0);
        idx1_pos++;
    }

    /* idx1 */
    wtag(f, "idx1");
    w32(f, a->frames * 16);
    if (fwrite(a->idx, 16, a->frames, f) != a->frames) {
        ESP_LOGE(TAG, "写索引失败");
    }

    long file_end = ftell(f);

    /* 索引真的写进去了，这才允许声明 AVIF_HASINDEX */
    patch32(f, a->off_avih_flags, 0x10);

    /* 实际帧率 */
    uint32_t uspf = (uint32_t)(elapsed_us / (a->frames ? a->frames : 1));
    if (uspf < 1000)  uspf = 1000;
    if (uspf > 1000000) uspf = 1000000;
    uint32_t fps = (uspf > 0) ? (1000000u / uspf) : 1;
    if (fps == 0) {
        fps = 1;
    }

    patch32(f, a->off_riff_size,        (uint32_t)(file_end - 8));
    patch32(f, a->off_avih_frames,      a->frames);
    patch32(f, a->off_avih_uspf,        uspf);
    patch32(f, a->off_avih_maxbps,      (uint32_t)(a->max_frame * fps));
    patch32(f, a->off_strh_length,      a->frames);
    patch32(f, a->off_strh_scale,       uspf);
    patch32(f, a->off_strh_rate,        1000000u);
    patch32(f, a->off_strh_suggested,   a->max_frame);
    patch32(f, a->off_movi_size,        (uint32_t)(idx1_pos - a->movi_fourcc_pos));

    fflush(f);
    fclose(f);
    a->fp = NULL;

    uint64_t bytes = (uint64_t)file_end;
    ESP_LOGI(TAG, "分段完成 %s：%u 帧 / %.1f fps / %.1f MB / 平均 %.0f KB每帧",
             a->path, (unsigned)a->frames,
             a->frames ? (1e6 / (double)uspf) : 0.0,
             (double)bytes / 1048576.0,
             a->frames ? (double)bytes / 1024.0 / (double)a->frames : 0.0);

    if (a->idx) {
        free(a->idx);
        a->idx = NULL;
    }
}

/* ================================================================== */
/* SD 挂载                                                             */
/* ================================================================== */

static esp_err_t mount_once(int width, int freq_khz, sdmmc_card_t **out)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = freq_khz;

    if (s_pwr == NULL) {
        sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = 4 };
        esp_err_t r = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &s_pwr);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "片内 LDO(通道4) 初始化失败: %s", esp_err_to_name(r));
            return r;
        }
        ESP_LOGI(TAG, "SD 供电 LDO(通道4) 已使能");
    }
    host.pwr_ctrl_handle = s_pwr;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_PIN_CLK;
    slot.cmd   = SD_PIN_CMD;
    slot.d0    = SD_PIN_D0;
    slot.width = width;
    if (width == 4) {
        slot.d1 = SD_PIN_D1;
        slot.d2 = SD_PIN_D2;
        slot.d3 = SD_PIN_D3;
    }
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot,
                                            &mount_cfg, out);
    if (ret != ESP_OK) {
        sdmmc_host_deinit();
    }
    return ret;
}

static bool sd_mount(void)
{
    ESP_LOGI(TAG, "挂载 SD 卡（CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d）",
             SD_PIN_CLK, SD_PIN_CMD, SD_PIN_D0, SD_PIN_D1, SD_PIN_D2, SD_PIN_D3);

    esp_err_t ret = mount_once(4, SDMMC_FREQ_HIGHSPEED, &s_card);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "[OK] 4 线 @40MHz 挂载成功");
    } else {
        ESP_LOGW(TAG, "4 线@40MHz 失败(%s)，退回 20MHz", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(100));

        ret = mount_once(4, SDMMC_FREQ_DEFAULT, &s_card);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[OK] 4 线 @20MHz 挂载成功");
        } else {
            ESP_LOGW(TAG, "4 线@20MHz 也失败(%s)，改 1 线", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = mount_once(1, SDMMC_FREQ_DEFAULT, &s_card);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "[OK] 1 线 @20MHz 挂载成功（速度会低一些）");
            }
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[FAIL] SD 挂载失败: %s", esp_err_to_name(ret));
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "    -> 卡片不应答：查卡是否插到底 / 是否 exFAT / 卡是否损坏");
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "    -> 卡能通信但文件系统挂不上：大概率是 exFAT，格式化成 FAT32");
        }
        return false;
    }

    sdmmc_card_print_info(stdout, s_card);
    s_mounted = true;
    return true;
}

/* ================================================================== */
/* 吞吐自测                                                            */
/* ================================================================== */

static void sd_selftest(void)
{
    static uint8_t *buf = NULL;
    const int CHUNK = 64 * 1024, NCHUNK = 8;      /* 512KB */

    if (buf == NULL) {
        buf = heap_caps_malloc(CHUNK, MALLOC_CAP_SPIRAM);
        if (buf == NULL) {
            return;
        }
        for (int i = 0; i < CHUNK; i++) {
            buf[i] = (uint8_t)(i * 37 + 11);
        }
    }

    const char *path = MOUNT_POINT "/tp.bin";
    int64_t t0 = esp_timer_get_time();
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "[FAIL] 写测试打不开文件 (errno=%d)", errno);
        return;
    }
    size_t total = 0;
    for (int i = 0; i < NCHUNK; i++) {
        total += fwrite(buf, 1, CHUNK, f);
    }
    fclose(f);
    int64_t t1 = esp_timer_get_time();

    int64_t us = (t1 > t0) ? (t1 - t0) : 1;
    double kb_s = (double)total / 1024.0 * 1000000.0 / (double)us;

    ESP_LOGI(TAG, "持续写: %u KB 用时 %.0f ms -> %.0f KB/s (%.2f MB/s)",
             (unsigned)(total / 1024), (double)us / 1000.0, kb_s, kb_s / 1024.0);

    /* 1080p 一帧约 100KB；20fps 需要约 2.0 MB/s */
    if (kb_s < 2000.0) {
        ESP_LOGW(TAG, "    -> 低于 2.0 MB/s，录制可能跟不上，建议降到 15fps 或开 H.264");
    } else {
        ESP_LOGI(TAG, "    -> 带宽够 1080p@20 录制（需要约 2.0 MB/s）");
    }

    remove(path);

    FATFS *fs = NULL;
    DWORD free_clusters = 0;
    if (f_getfree("0:", &free_clusters, &fs) == FR_OK && fs != NULL) {
        uint64_t free_mb = (uint64_t)free_clusters * fs->csize / 2 / 1024;
        ESP_LOGI(TAG, "剩余空间: %llu MB", (unsigned long long)free_mb);
    }
}

/* ================================================================== */
/* 录制                                                                */
/* ================================================================== */

static uint64_t sd_free_mb(void)
{
    FATFS *fs = NULL;
    DWORD fc = 0;
    if (f_getfree("0:", &fc, &fs) == FR_OK && fs != NULL) {
        return (uint64_t)fc * fs->csize / 2 / 1024;
    }
    return 0;
}

/* 删掉最旧的一个 .avi，成功返回 true */
static bool delete_oldest_video(void)
{
    DIR *d = opendir(VIDEO_DIR);
    if (d == NULL) {
        return false;
    }

    char oldest[64] = { 0 };
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        size_t l = strlen(e->d_name);
        if (l < 4 || strcasecmp(e->d_name + l - 4, ".avi") != 0) {
            continue;
        }
        if (oldest[0] == 0 || strcmp(e->d_name, oldest) < 0) {
            size_t c = l > 63 ? 63 : l;
            memcpy(oldest, e->d_name, c);
            oldest[c] = 0;
        }
    }
    closedir(d);

    if (oldest[0] == 0) {
        return false;
    }

    char path[96];
    snprintf(path, sizeof(path), VIDEO_DIR "/%s", oldest);
    if (remove(path) == 0) {
        ESP_LOGI(TAG, "空间不足，删除旧分段 %s", oldest);
        return true;
    }
    ESP_LOGE(TAG, "删除 %s 失败 (errno=%d)", path, errno);
    return false;
}

/* 保证至少 need_mb 的可用空间；拿不到返回 false */
static bool ensure_space(uint64_t need_mb)
{
    int guard = 0;
    while (sd_free_mb() < need_mb) {
        if (!delete_oldest_video() || ++guard > 200) {
            ESP_LOGE(TAG, "清理后空间仍不足（剩 %llu MB / 需要 %llu MB）",
                     (unsigned long long)sd_free_mb(),
                     (unsigned long long)need_mb);
            return false;
        }
    }
    return true;
}

static void video_record_loop(void *arg)
{
    const size_t FRAME_CAP = 320 * 1024;
    uint8_t *frame = heap_caps_malloc(FRAME_CAP, MALLOC_CAP_SPIRAM);
    if (frame == NULL) {
        ESP_LOGE(TAG, "帧缓冲分配失败，录制不启动");
        vTaskDelete(NULL);
        return;
    }

    struct stat st;
    if (stat(VIDEO_DIR, &st) != 0) {
        if (mkdir(VIDEO_DIR, 0755) == 0) {
            ESP_LOGI(TAG, "已创建 %s", VIDEO_DIR);
        }
    }

    const uint32_t seg_ms   = (uint32_t)CONFIG_VIDEO_SEGMENT_DURATION_MIN * 60u * 1000u;
    const uint32_t seg_frames = CONFIG_VIDEO_SEGMENT_DURATION_MIN * 60u * REC_POLL_HZ;
    uint32_t file_no = 0;
    uint32_t open_fail = 0;

    ESP_LOGI(TAG, "录制任务启动（轮询 %d Hz，逐帧录，%d 分钟或 %u MB 一段）",
             REC_POLL_HZ, CONFIG_VIDEO_SEGMENT_DURATION_MIN,
             (unsigned)(SEG_MAX_BYTES / 1048576u));
    ESP_LOGI(TAG, "空间策略：每段开始前保证 256MB 可用，不足则删最旧的 .avi");

    while (1) {
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);

        char path[96];
        snprintf(path, sizeof(path), VIDEO_DIR "/%04d%02d%02d_%02d%02d%02d_%03u.avi",
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min, tm.tm_sec, (unsigned)(file_no++ % 1000));

        /* 每段开始前保证 1GB 可用；清理失败就不要开新文件，避免产生坏录像 */
        /* 每段开始前预留空间：卡里旧数据多时 1GB 门槛会让录制一直起不来，
         * 降到 256MB（≈4 分钟 1080p）后再靠删旧分段滚动 */
        if (!ensure_space(256)) {
            ESP_LOGE(TAG, "空间不足且无法清理，暂停录制 60 秒后重试");
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        avi_t avi;
        if (!avi_open(&avi, path, 1920, 1080, seg_frames)) {
            /* 卡坏了会一直失败：退避重试，别每 3 秒刷一行日志 */
            open_fail++;
            uint32_t wait_ms = 3000;
            if (open_fail >= 5) {
                wait_ms = 60000;
                if (open_fail % 10 == 5) {
                    ESP_LOGE(TAG, "SD 卡连续 %u 次无法创建文件，改 60 秒重试"
                                  "（卡可能已损坏，建议换卡）", open_fail);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
            continue;
        }
        open_fail = 0;
        ESP_LOGI(TAG, "开始录制 %s（剩余 %llu MB）", path,
                 (unsigned long long)sd_free_mb());

        s_recording = true;
        uint32_t last_seq = 0;
        int64_t  t_start = esp_timer_get_time();
        int64_t  next_frame_us = t_start;      /* 绝对时间节拍 */
        bool     have_any = false;

        while (1) {
            if (avi.frames >= seg_frames) {
                ESP_LOGI(TAG, "到达帧数上限，换段");
                break;
            }
            if ((esp_timer_get_time() - t_start) / 1000 >= (int64_t)seg_ms) {
                ESP_LOGI(TAG, "到达时长上限，换段");
                break;
            }
            if (ftell(avi.fp) >= (long)SEG_MAX_BYTES) {
                ESP_LOGI(TAG, "到达体积上限，换段");
                break;
            }

            size_t len = 0;
            uint32_t seq = 0;
            if (camera_copy_latest_frame(frame, FRAME_CAP, &len, &seq) == ESP_OK &&
                seq != last_seq && len > 0) {
                last_seq = seq;
                have_any = true;
                if (!avi_add_frame(&avi, frame, len)) {
                    ESP_LOGE(TAG, "写帧失败（可能是磁盘满），提前结束本段");
                    break;
                }

                /* 每 30 帧落盘一次：10fps 下约 3 秒，断电最多丢这么久 */
                if (avi.frames % 30 == 0) {
                    avi_sync(&avi);

                    /* 对时到达后日期会从 1970 跳到真实日期：立刻换段，
                     * 否则整段（最长 30 分钟）文件名都是错的 */
                    time_t tn = time(NULL);
                    struct tm tmn;
                    localtime_r(&tn, &tmn);
                    if (tmn.tm_year != tm.tm_year || tmn.tm_yday != tm.tm_yday) {
                        ESP_LOGI(TAG, "日期已更新（对时生效），结束本段重新命名");
                        break;
                    }
                }

                if (avi.frames % 200 == 0) {
                    double sec = (double)(esp_timer_get_time() - t_start) / 1e6;
                    ESP_LOGI(TAG, "录制中 %u 帧 / %.1f MB / %.1f fps",
                             (unsigned)avi.frames,
                             (double)ftell(avi.fp) / 1048576.0,
                             sec > 0 ? avi.frames / sec : 0.0);
                }
            }

            /* 绝对时间节拍：只补足到下一帧的时间点，写慢了就不睡。
             * 原来的固定 50ms 死等会让写满一帧后还空转，实测只有 9fps。 */
            next_frame_us += (1000000 / REC_POLL_HZ);
            int64_t wait_us = next_frame_us - esp_timer_get_time();
            if (wait_us > 2000) {
                vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
            } else if (wait_us > 0) {
                esp_rom_delay_us((uint32_t)wait_us);
            } else {
                next_frame_us = esp_timer_get_time();   /* 落后太多，重新对齐 */
            }
        }

        s_recording = false;
        avi_close(&avi);

        if (!have_any) {
            ESP_LOGW(TAG, "这一段落没有取到任何帧，删除空文件");
            remove(path);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

    }
}

/* ================================================================== */
/* 对外接口                                                            */
/* ================================================================== */

bool sd_card_is_mounted(void)
{
    return s_mounted;
}

bool video_is_recording(void)
{
    return s_recording;
}


/* 把某个目录下指定后缀的文件追加到 JSON 里，返回新的偏移 */
static int list_dir(const char *dir, const char *ext, char *out, int max,
                    int off, bool *first)
{
    DIR *d = opendir(dir);
    if (d == NULL) {
        return off;
    }

    struct dirent *e;
    while ((e = readdir(d)) != NULL && off < max - 160) {
        size_t l = strlen(e->d_name);
        size_t xl = strlen(ext);
        if (l <= xl || strcasecmp(e->d_name + l - xl, ext) != 0) {
            continue;
        }

        char path[128];
        struct stat st;
        snprintf(path, sizeof(path), "%s/%.64s", dir, e->d_name);
        if (stat(path, &st) != 0) {
            continue;
        }

        off += snprintf(out + off, max - off,
                        "%s{\"name\":\"%.64s\",\"bytes\":%llu}",
                        *first ? "" : ",", e->d_name,
                        (unsigned long long)st.st_size);
        *first = false;
    }
    closedir(d);
    return off;
}

/* 列出录像与日志文件（供 /sd 接口查看写入情况） */
int video_list_files(char *out, int max)
{
    int off = snprintf(out, max,
                       "{\"ok\":1,\"mount\":%d,\"rec\":%d,\"free_mb\":%llu,\"files\":[",
                       s_mounted ? 1 : 0, s_recording ? 1 : 0,
                       (unsigned long long)sd_free_mb());

    bool first = true;
    off = list_dir(VIDEO_DIR,      ".avi", out, max, off, &first);
    off = list_dir(MOUNT_POINT "/logs", ".csv", out, max, off, &first);

    off += snprintf(out + off, max - off, "]}\n");
    return off;
}

void video_record_task_init(void)
{
    ESP_LOGI(TAG, "===== SD 卡初始化 =====");

    if (!sd_mount()) {
        ESP_LOGW(TAG, "SD 不可用，录制与日志功能跳过；系统其余部分正常运行");
        return;
    }

    sd_selftest();

    xTaskCreatePinnedToCore(video_record_loop, "video_record",
                            8192, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "[OK] SD 初始化完成");
}
