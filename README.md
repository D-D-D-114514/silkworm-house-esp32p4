# ESP32-P4 智能蚕房监测系统 · P4 端固件

基于 **ESP32-P4-Module-DEV-KIT** 的智能蚕房监测管控系统 —— 视觉感知、视频留存、跨板通信与决策层。

> 完整交接文档在 **[`docs/`](docs/README.md)**。本 README 只做快速入口。

---

## 功能

- **1080p 采集**：OV5647 MIPI-CSI 2-lane → ISP → JPEG（1920×1080 1:1 裁切，无插值）
- **帧率/曝光可调**：VTS 3600 → **10 fps / 曝光上限 100 ms**（用帧率换信噪比）
- **网页推流**：`http://192.168.1.10/` MJPEG，异步 handler 不阻塞控制台
- **SD 录像**：MJPEG AVI，30 分钟或 512 MB 分段，1 GB 空间水位自动删最旧
- **双流日志**：遥测与事件写同一 CSV，共用时间戳（便于确定因果）
- **跨板通信**：UART2 115200 JSON 行协议，半帧/离线/丢包全可诊断
- **AI 决策**：PC 推 `POST /ai` → P4 规则引擎 → 下发 STM32 命令
- **PC 抓流工具**：`tools/` 下图形界面 + 命令行两种形态

## 快速开始

```bash
# 1. 构建
powershell -ExecutionPolicy Bypass -File scripts/build.ps1

# 2. 烧录（COM11）
idf.py -p COM11 flash monitor

# 3. 浏览器打开控制台
http://192.168.1.10/
```

PC 网卡需为 `192.168.1.100/24`（板子是静态 `192.168.1.10`）。

## 录制视频的两种方式

| 方式 | 怎么用 | 适合 |
|---|---|---|
| **PC 抓流** | 双击桌面「蚕视频录制器」，或 `python tools/p4_recorder.py` | 做数据集（不掉帧、头部正确、直接可用） |
| **SD 卡** | 上电自动开始 | 断电保险（不依赖 PC） |

两者可同时开 —— 读的是同一份 JPEG，网口占用仅约 10%。

## 引脚速查（详细见 [docs/接线总表.md](docs/接线总表.md)）

| 外设 | 引脚 |
|---|---|
| 摄像头 SCCB | SDA=7, SCL=8；MIPI 2-lane；PHY 供电 = 片内 LDO 通道 3 |
| MicroSD | CLK=43, CMD=44, D0–D3=39/40/41/42；供电 = **片内 LDO 通道 4** |
| 以太网 | TX_EN=49, TXD0/1=34/35, CRS_DV=28, RXD0/1=29/30, MDC=31, MDIO=52, PHY_RST=51, REF_CLK=50 |
| STM32 UART2 | **TX=22 → PA3**，**RX=21 ← PA2**，GND |
| 保留 | **GPIO37/38 = console UART，不可占用** |

## 文档索引

| 文件 | 内容 |
|---|---|
| [docs/README.md](docs/README.md) | 文档索引 + 三条最易踩的坑 |
| [docs/01-项目交接.md](docs/01-项目交接.md) | 架构、目录、参数、引脚、构建、当前状态、缺陷、技术决策记录 |
| [docs/02-论文素材.md](docs/02-论文素材.md) | 技术贡献点、实测数据、图表建议、对比实验设计、局限性 |
| [docs/03-接口手册.md](docs/03-接口手册.md) | HTTP 端点、UART 协议、日志字段、AI 规则 |
| [docs/接线总表.md](docs/接线总表.md) | 完整接线（含电源、继电器、MOS 管、TEC/PTC） |

## 环境

| 项 | 版本 / 位置 |
|---|---|
| ESP-IDF | **v6.1**（`D:\.espressif\v6.1\esp-idf`，旧 `C:\esp\v6.1\esp-idf` 仍在） |
| 工具链 | `C:\Espressif\tools`，`riscv32-esp-elf-gcc 15.2.0` |
| 串口 | COM11 @115200 |
| 依赖 | `esp_cam_sensor ^2.1`、`esp_video 2.2~3.0`、`esp_h264 1.4.1`（未启用） |

## 致谢 / 参考

- [Waveshare ESP32-P4-Platform 示例](https://github.com/waveshareteam/ESP32-P4-Platform)
- ESP-IDF `esp_video` / `esp_cam_sensor` 组件文档

## License

MIT

## 许可证

本项目采用 [MIT License](LICENSE)。

仓库内含第三方组件：ESP-IDF 及其组件（Apache-2.0）、ST HAL/CMSIS（BSD-3-Clause），
版权归各自作者，按其原许可证分发。
