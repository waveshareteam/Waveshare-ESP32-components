# BSP：ESP32-P4-WIFI6-DB

[English Version](./README.md)

## 概述

本组件提供 ESP32-P4-WIFI6-DB 的板级支持包。当前组件清单版本为 `0.0.1`。

支持的板载接口：

- 4-bit SDMMC 模式的 SD 卡。
- ES8311 播放和一路模拟麦克风输入。
- 共享 I2C 总线。
- 可选的双 lane MIPI-DSI 显示屏，支持 JD9365、ILI9881C 或 HX8394。
- GT911 电容触摸控制器。
- 通过 `esp_video` 接入的 MIPI-CSI 摄像头。
- USB Host。
- 扩展排针 GPIO 枚举。

本板不提供外部 RTC，也没有 LCD 复位 GPIO。音频功放由 GPIO53 使能。GT911 的复位
和中断信号均未连接。

## 引脚分配

### I2C

| 信号 | GPIO |
| --- | ---: |
| SDA | 7 |
| SCL | 8 |

ES8311、GT911、LCD 背光控制器和摄像头 SCCB 总线共享这条 I2C 总线。

### ES8311

| ES8311 信号 | ESP32-P4 信号 | GPIO |
| --- | --- | ---: |
| MCLK | I2S MCLK | 13 |
| SCLK | I2S BCLK | 12 |
| LRCK | I2S WS | 10 |
| DSDIN | I2S DOUT | 9 |
| ASDOUT | I2S DIN | 11 |
| 功放使能 | GPIO | 53 |

`bsp_audio_codec_speaker_init()` 和 `bsp_audio_codec_microphone_init()` 都会创建
ES8311 codec 设备。麦克风通路按一路模拟麦克风配置。

### SD 卡

| 信号 | GPIO |
| --- | ---: |
| D0 | 39 |
| D1 | 40 |
| D2 | 41 |
| D3 | 42 |
| CMD | 44 |
| CLK | 43 |
| 电源使能 | 45，低有效 |

`bsp_sdcard_mount()` 会先断开卡电源 100 ms，重新上电后再等待 200 ms，然后才开始
SDMMC 初始化。在 ESP32-P4 上，SDMMC 的 IO 电压通过片内 LDO 通道 4 控制。BSP 还会
使能 SDMMC 内部上拉，但要保证信号完整性仍需外部上拉。

### 显示

在 `menuconfig` 中选择实际连接的面板，默认是 10.1 英寸 JD9365。

| 面板选项 | 控制器 | 分辨率 | DSI lane 速率 |
| --- | --- | ---: | ---: |
| Waveshare 5-DSI-TOUCH-A | HX8394 | 720x1280 | 700 Mbps |
| Waveshare 7-DSI-TOUCH-A | ILI9881C | 720x1280 | 1000 Mbps |
| Waveshare 8-DSI-TOUCH-A | JD9365 | 800x1280 | 1500 Mbps |
| Waveshare 10.1-DSI-TOUCH-A | JD9365 | 800x1280 | 1500 Mbps |

四个选项都使用两条 DSI 数据 lane。LCD 复位引脚为 `GPIO_NUM_NC`。

背光亮度通过共享 I2C 总线控制：

| 项目 | 取值 |
| --- | --- |
| I2C 地址 | `0x45` |
| 亮度寄存器 | `0x96` |
| 亮度数据 | `0..255`，由 `0..100%` 换算得到 |

`bsp_display_brightness_set()` 发送 `{0x96, data}`，其中
`data = 255 * brightness_percent / 100`。

### 触摸

GT911 使用 GPIO7/GPIO8 上的共享 I2C 总线。它的复位和中断 GPIO 都是 `GPIO_NUM_NC`，
因此驱动采用轮询方式。`bsp_touch_new()` 会依次尝试 GT911 的两个有效地址 `0x5D`
和 `0x14`。

`bsp_display_start()` 和 `bsp_display_start_with_config()` 会自动创建 GT911 设备
并注册到 LVGL 适配层。

### 扩展排针

`bsp_get_header_gpios()` 返回下面这个由 BSP 持有的只读数组：

```text
52, 51, 31, 30, 29, 28, 50, 49, 5, 4, 3, 2, 24,
25, 20, 21, 22, 23, 26, 27, 32, 33, 46, 47, 48
```

## 配置

在 `menuconfig` 中可以选择：

- I2C 控制器以及 100/400 kHz 总线速率。
- I2S 控制器。
- 连接的 MIPI-DSI 面板类型。
- RGB565 或 RGB888 显示格式。
- 一到三个 MIPI-DPI 帧缓冲。
- SD 卡和 SPIFFS 挂载选项。

## 基本用法

```c
#include "bsp/esp-bsp.h"

ESP_ERROR_CHECK(bsp_i2c_init());

esp_codec_dev_handle_t playback = bsp_audio_codec_speaker_init();
esp_codec_dev_handle_t microphone = bsp_audio_codec_microphone_init();

esp_lcd_panel_handle_t panel = NULL;
esp_lcd_panel_io_handle_t io = NULL;
ESP_ERROR_CHECK(bsp_display_new(NULL, &panel, &io));
ESP_ERROR_CHECK(bsp_display_brightness_set(50));

esp_lcd_touch_handle_t touch = NULL;
ESP_ERROR_CHECK(bsp_touch_new(NULL, &touch));
```

当前 BSP 的改动已在 ESP-IDF v6.0.1 上烧录验证过。该验证不能代替针对显示输出、
音频播放/采集、SD 卡操作、USB、CSI、触摸输入和电气特性的专项检查。
