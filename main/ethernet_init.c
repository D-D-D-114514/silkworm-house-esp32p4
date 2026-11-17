/*
 * 以太网初始化
 * ESP32-P4-Module-DEV-KIT 板载 RJ45 网口
 */

#include <string.h>
#include <inttypes.h>

#include "ethernet_init.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "ping/ping_sock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"

static const char *TAG = "ETH";

#define ETH_STATIC_IP   "192.168.1.10"   /* 板子 */
#define ETH_PEER_IP     "192.168.1.100"  /* 电脑 */

static volatile bool    s_link_up = false;
static volatile bool    s_ping_done = false;
static esp_eth_handle_t s_eth_handle = NULL;

/* 打印 PHY 协商出来的速率/双工，速率不对就是网线或网卡节能设置的问题 */
static void log_link_negotiation(void)
{
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;

    if (s_eth_handle == NULL) {
        return;
    }
    if (esp_eth_ioctl(s_eth_handle, ETH_CMD_G_SPEED, &speed) == ESP_OK &&
        esp_eth_ioctl(s_eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex) == ESP_OK) {
        ESP_LOGI(TAG, "协商结果: %s / %s",
                 speed == ETH_SPEED_100M ? "100M" : "10M",
                 duplex == ETH_DUPLEX_FULL ? "全双工" : "半双工");
        if (speed != ETH_SPEED_100M || duplex != ETH_DUPLEX_FULL) {
            ESP_LOGW(TAG, "速率/双工异常：查网线质量、电脑网卡的节能以太网(EEE)");
        }
    } else {
        ESP_LOGW(TAG, "读取速率/双工失败");
    }
}

// 事件处理器
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link_up = true;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        log_link_negotiation();
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/* ---------------- 连通性自检：板子 -> 电脑 ---------------- */

/* 读 PHY 关键寄存器 + dump EMAC 寄存器。
 * 判据：若 MACSTATUS 显示的速度/双工与 PHY 不一致或为 0，
 *       说明 MAC 没收到 REF_CLK（GPIO50 上没有 50MHz），
 *       此时 MDIO 仍能工作、链路仍能协商，但数据帧发不出去。 */
static void eth_dump_diagnostics(void)
{
    static const struct {
        uint16_t    reg;
        const char *name;
    } regs[] = {
        { 0x00, "BMCR   " }, { 0x01, "BMSR   " }, { 0x02, "PHYID1 " },
        { 0x03, "PHYID2 " }, { 0x04, "ANAR   " }, { 0x05, "ANLPAR " },
        { 0x10, "PHYSTS " }, { 0x11, "SPEC11 " }, { 0x16, "SPEC16 " },
        { 0x1E, "SPEC1E " },
    };

    ESP_LOGI("DIAG", "===== PHY 寄存器 =====");
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        uint32_t val = 0;
        esp_eth_phy_reg_rw_data_t rw = {
            .reg_addr = regs[i].reg,
            .reg_value_p = &val,
        };
        if (esp_eth_ioctl(s_eth_handle, ETH_CMD_READ_PHY_REG, &rw) == ESP_OK) {
            ESP_LOGI("DIAG", "PHY[0x%02X] %s = 0x%04" PRIx32,
                     regs[i].reg, regs[i].name, val);
        } else {
            ESP_LOGW("DIAG", "PHY[0x%02X] %s 读取失败", regs[i].reg, regs[i].name);
        }
    }

    ESP_LOGI("DIAG", "===== EMAC 寄存器 dump（先看 MAC 段）=====");
    esp_eth_ioctl(s_eth_handle, ETH_MAC_ESP_CMD_DUMP_REGS, NULL);
}

static void on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno = 0;
    uint32_t rtt = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &rtt, sizeof(rtt));
    ESP_LOGI("PING", "[OK] 收到电脑应答 seq=%u 往返=%" PRIu32 "ms",
             (unsigned)seqno, rtt);
}

static void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    ESP_LOGW("PING", "[..] 超时 seq=%u", (unsigned)seqno);
}

static void on_ping_end(esp_ping_handle_t hdl, void *args)
{
    uint32_t sent = 0, recv = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &sent, sizeof(sent));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &recv, sizeof(recv));

    if (recv > 0) {
        ESP_LOGI("PING", "自检结论: 板子->电脑 通 (收 %" PRIu32 "/%" PRIu32
                 ")，物理层与 IP 配置正常，问题在 Windows 侧", recv, sent);
    } else {
        ESP_LOGE("PING", "自检结论: 板子->电脑 不通 (0/%" PRIu32
                 ")，问题在板子侧：RMII 数据面/速率双工/网线", sent);
    }
    esp_ping_delete_session(hdl);
    s_ping_done = true;
}

/* 用 PCNT 硬件计数，精确测 GPIO50 的频率。
 * 判据：RMII 模式下 PHY 的 50M_CLKO 输出 50MHz；
 *       若测到约 25MHz，说明 PHY 处于 MII 模式（该脚是 RXCLK 25MHz），
 *       MAC 会以半速时钟跑 RMII，发出去的帧全是坏的 —— 与现象吻合。 */
static void measure_refclk_hz(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit = -1,
        .high_limit = 30000,
        .flags.accum_count = 0,
    };
    pcnt_unit_handle_t unit = NULL;
    if (pcnt_new_unit(&ucfg, &unit) != ESP_OK) {
        ESP_LOGE("DIAG", "PCNT 单元创建失败");
        return;
    }

    pcnt_chan_config_t ccfg = { .edge_gpio_num = 50, .level_gpio_num = -1 };
    pcnt_channel_handle_t chan = NULL;
    if (pcnt_new_channel(unit, &ccfg, &chan) != ESP_OK) {
        ESP_LOGE("DIAG", "PCNT 通道创建失败");
        pcnt_del_unit(unit);
        return;
    }
    /* 上升沿 +1，下降沿不动 → 计数值 = 周期数 */
    pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_HOLD);

    pcnt_unit_enable(unit);
    pcnt_unit_clear_count(unit);
    pcnt_unit_start(unit);

    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 500) {
        /* 忙等 500us，避免 tick 粒度误差 */
    }
    int count = 0;
    pcnt_unit_get_count(unit, &count);
    int64_t t1 = esp_timer_get_time();
    pcnt_unit_stop(unit);
    pcnt_unit_disable(unit);

    double sec = (double)(t1 - t0) / 1000000.0;
    double hz = sec > 0 ? count / sec : 0;

    ESP_LOGI("DIAG", "PCNT 测频: %d 个周期 / %.0f us = %.2f MHz",
             count, sec * 1e6, hz / 1e6);
    if (hz > 40000000) {
        ESP_LOGI("DIAG", "结论: REF_CLK ≈ 50MHz，PHY 处于 RMII 模式，时钟正确");
    } else if (hz > 20000000) {
        ESP_LOGE("DIAG", "结论: REF_CLK ≈ 25MHz！PHY 很可能处于 MII 模式，"
                 "MAC 被半速时钟驱动，数据面必然失效");
    } else if (hz > 1000) {
        ESP_LOGW("DIAG", "结论: REF_CLK 频率异常 (%.2f MHz)", hz / 1e6);
    } else {
        ESP_LOGE("DIAG", "结论: GPIO50 上没有可用时钟");
    }
}

/* 把 GPIO50 当普通输入采样：
 * REF_CLK 是 50MHz（周期 20ns），CPU 逐次读取必然采到 0/1 混合并产生大量跳变。
 * 如果采样结果恒为同一个电平、跳变为 0，说明 PHY 根本没有输出 REF_CLK，
 * 此时 MDIO 仍能工作、链路仍能协商，但数据面完全死掉 —— 与现象完全吻合。 */
static void probe_refclk(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << 50,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    int ones = 0, zeros = 0, trans = 0, last = -1;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < 50000; i++) {
        int v = gpio_get_level(50);
        if (v) {
            ones++;
        } else {
            zeros++;
        }
        if (last >= 0 && v != last) {
            trans++;
        }
        last = v;
    }
    int64_t t1 = esp_timer_get_time();
    double dur_us = (double)(t1 - t0);

    ESP_LOGI("DIAG", "GPIO50 采样 50000 次: 高=%d 低=%d 跳变=%d 耗时=%.0fus 估算=%.1fMHz",
             ones, zeros, trans, dur_us, dur_us > 0 ? trans / (2.0 * dur_us) : 0.0);
    if (trans > 200) {
        ESP_LOGI("DIAG", "结论: GPIO50 上有高频时钟，REF_CLK 正常送达 MAC");
    } else {
        ESP_LOGE("DIAG", "结论: GPIO50 上没有时钟（静态电平）！REF_CLK 未送达，MAC 无法收发");
    }
}

/* 读 RMII 数据面引脚的 IO_MUX 实际配置。
 * 数据线必须走 IO_MUX 专用功能（fun_sel != 0），
 * 若 fun_sel 仍是 0（普通 GPIO），说明 EMAC 的引脚复用没生效，
 * 数据线是哑的 —— 此时 MDIO 正常、链路正常、但一个帧都出不去。 */
static void dump_rmii_iomux(void)
{
    static const struct {
        int         pin;
        const char *name;
    } pins[] = {
        { 28, "CRS_DV" }, { 29, "RXD0  " }, { 30, "RXD1  " },
        { 34, "TXD0  " }, { 35, "TXD1  " }, { 49, "TX_EN " },
        { 50, "REF_CLK" }, { 31, "MDC   " }, { 52, "MDIO  " },
    };

    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_io_config_t cfg = { 0 };
        gpio_ll_get_io_config(&GPIO, pins[i].pin, &cfg);
        ESP_LOGI("DIAG", "IO_MUX GPIO%-2d %s: fun_sel=%" PRIu32 " oe=%d ie=%d sig_out=%" PRIu32,
                 pins[i].pin, pins[i].name,
                 (uint32_t)cfg.fun_sel, cfg.oe, cfg.ie, (uint32_t)cfg.sig_out);
    }
}

/* 直接读 EMAC DMA 的当前 RX 描述符指针（偏移 0x4C）。
 * RX 每收到一帧，DMA 就会推进这个指针 —— 它是判断"板子到底有没有收到帧"
 * 最直接的证据，不依赖任何上层协议栈和电脑侧配合。 */
#define EMAC_DMA_BASE 0x50099000u
static uint32_t emac_dma_reg(uint32_t off)
{
    return *(volatile uint32_t *)(EMAC_DMA_BASE + off);
}

static void eth_selftest_task(void *arg)
{
    for (int i = 0; i < 60 && !s_link_up; i++) {   /* 最多等 30s */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!s_link_up) {
        ESP_LOGW("PING", "链路未建立，跳过自检（先查网线/网口）");
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    ip_addr_t target;
    memset(&target, 0, sizeof(target));
    if (!ipaddr_aton(ETH_PEER_IP, &target)) {
        ESP_LOGE("PING", "目标地址解析失败");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("PING", "持续自检: 板子 %s <-> 电脑 %s", ETH_STATIC_IP, ETH_PEER_IP);

    uint32_t last_rx_desc = emac_dma_reg(0x4C);
    ESP_LOGI("DIAG", "初始 RX 描述符指针 = 0x%08" PRIx32, last_rx_desc);

    /* 开机自检 3 轮即可；之后退出，避免刷屏干扰后续调试 */
    for (int round = 0; round < 3; round++) {
        esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
        cfg.count = 4;
        cfg.interval_ms = 1000;
        cfg.timeout_ms = 1500;
        cfg.data_size = 32;
        cfg.target_addr = target;

        esp_ping_callbacks_t cbs = {
            .cb_args = NULL,
            .on_ping_success = on_ping_success,
            .on_ping_timeout = on_ping_timeout,
            .on_ping_end = on_ping_end,
        };

        s_ping_done = false;
        esp_ping_handle_t ping = NULL;
        if (esp_ping_new_session(&cfg, &cbs, &ping) == ESP_OK) {
            if (esp_ping_start(ping) != ESP_OK) {
                esp_ping_delete_session(ping);
            }
        } else {
            ESP_LOGE("PING", "创建 ping 会话失败");
        }

        for (int i = 0; i < 120 && !s_ping_done; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uint32_t rx_desc = emac_dma_reg(0x4C);
        ESP_LOGI("DIAG", "RX 描述符指针=0x%08" PRIx32 " %s   DMA状态=0x%08" PRIx32,
                 rx_desc,
                 rx_desc != last_rx_desc ? "[收到过帧]" : "[一帧都没收到]",
                 emac_dma_reg(0x14));
        last_rx_desc = rx_desc;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI("PING", "以太网自检结束");
    vTaskDelete(NULL);
}

void ethernet_init(void)
{
    ESP_LOGI(TAG, "Initializing Ethernet...");

    // 创建默认网络接口
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    // 配置静态 IP（不使用 DHCP）
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(eth_netif));  // 停止 DHCP 客户端
    
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 10);           // 板子 IP: 192.168.1.10
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 100);          // 网关: 192.168.1.100 (电脑)
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);     // 子网掩码
    ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &ip_info));
    
    ESP_LOGI(TAG, "Static IP configured: 192.168.1.10");

    // EMAC：显式写死已验证的 RMII 引脚（IP101GRI，REF_CLK 由 PHY 送进 GPIO50）
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = {
        .smi_gpio = { .mdc_num = 31, .mdio_num = 52 },
        .interface = EMAC_DATA_INTERFACE_RMII,
        .clock_config = {
            .rmii = {
                .clock_mode = EMAC_CLK_EXT_IN,   // 50MHz 由 PHY 提供
                .clock_gpio = 50,
            }
        },
        .dma_burst_len = ETH_DMA_BURST_LEN_32,
        .intr_priority = 0,
        .emac_dataif_gpio = {
            .rmii = {
                .tx_en_num  = 49,
                .txd0_num   = 34,
                .txd1_num   = 35,
                .crs_dv_num = 28,
                .rxd0_num   = 29,
                .rxd1_num   = 30,
            }
        },
        .clock_config_out_in = {
            .rmii = { .clock_mode = EMAC_CLK_EXT_IN, .clock_gpio = -1 }
        },
        .mdc_freq_hz = 0,
    };
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);

    // PHY：IP101GRI，地址 1，复位脚 GPIO51
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_ETHERNET_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_ETHERNET_PHY_RST_GPIO;
    phy_config.hw_reset_assert_time_us = 10000;   // 复位脉冲 10ms
    phy_config.post_hw_reset_delay_ms = 150;      // 复位后等 150ms
    phy_config.autonego_timeout_ms = 5000;

    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);

    // 以太网配置
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    // 绑定网络接口
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(s_eth_handle)));

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    
    // 不再需要 IP_EVENT_ETH_GOT_IP 事件（因为使用静态 IP）

    // 启动以太网
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    ESP_LOGI(TAG, "[✓] Ethernet initialized");

    /* 开混杂模式：排除 MAC 地址过滤导致的收包丢失 */
    bool promisc = true;
    if (esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &promisc) == ESP_OK) {
        ESP_LOGI(TAG, "已开启混杂模式（排除地址过滤丢包）");
    }

    // 连通性自检任务（板子主动 ping 电脑）
    xTaskCreate(eth_selftest_task, "eth_selftest", 4096, NULL, 4, NULL);
}
