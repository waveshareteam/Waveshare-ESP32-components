# BSP：Waveshare ESP32-C6-Touch-AMOLED-2.16

[English](README.md)

这是适用于 Waveshare
[ESP32-C6-Touch-AMOLED-2.16](https://www.waveshare.com/esp32-c6-touch-amoled-2.16.htm)
的可复用 ESP-IDF 板级支持包。板载 ESP32-C6、480 × 480 SH8601 QSPI
AMOLED、CST9217 触摸、AXP2101 PMU、QMI8658 IMU、PCF85063 RTC、ES8311
播放编解码器、ES7210 录音编解码器和 microSD 卡槽。

## 公共 API

使用统一头文件：

~~~c
#include "bsp/esp-bsp.h"
~~~

通过 BSP 初始化共享总线和板载设备：

~~~c
ESP_ERROR_CHECK(bsp_pmu_init());
i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();

lv_display_t *display = bsp_display_start();
assert(display != NULL);

ESP_ERROR_CHECK(bsp_sdcard_mount());
esp_codec_dev_handle_t speaker = bsp_audio_codec_speaker_init();
esp_codec_dev_handle_t microphone = bsp_audio_codec_microphone_init();
~~~

BSP 提供共享 I2C0 句柄以及 AXP2101（0x34）、QMI8658（0x6B）和
PCF85063（0x51）的公共地址。应用应继续使用托管的 QMI8658 和 PCF85063
驱动，不再复制板级 I2C 封装。

`bsp_pmu_get_snapshot()` 可读取电池存在状态、充电状态和电池电压。
AMOLED 复位/供电由 AXP2101 ALDO3 控制，音频功放由 ALDO2 控制；根据产品
原理图，ALDO4 被配置为 1.8 V。

## 共享 SPI2 总线

显示屏使用 GPIO0-4 和 CS GPIO15 的 QSPI 接口。microSD 复用 GPIO0
（时钟）、GPIO1（MOSI）、GPIO2（MISO），CS 为 GPIO6。BSP 只初始化一次
支持 QSPI 的 SPI2 总线，再把显示屏和 SD 卡作为独立设备加入，因此不会
重复调用 `spi_bus_initialize()`。

显示、LVGL 适配层、音频通道和共享 SPI 总线均按应用生命周期的单例资源
设计。应用任务调用 LVGL API 时必须使用 `bsp_display_lock()` 与
`bsp_display_unlock()` 加锁。

## 依赖方式

组件尚未正式发布前，在产品仓库中锁定 BSP PR 对应的完整提交 SHA：

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_2_16:
    git: https://github.com/waveshareteam/Waveshare-ESP32-components.git
    path: bsp/esp32_c6_touch_amoled_2_16
    version: "<full-component-commit-sha>"
~~~

1.0.0 发布到组件服务后改用正式版本：

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_2_16: "^1.0.0"
~~~

版本 1.0.0 要求 ESP-IDF >=5.5、LVGL >=8,<10、`esp_codec_dev ~1.5`、
`waveshare/esp_lcd_sh8601 ^2.0.0`、
`waveshare/esp_lcd_touch_cst9217 ^2.0.0` 和
`espressif/esp_lvgl_adapter ~0.6`。目标芯片为 `esp32c6`，仓库 CI 覆盖
ESP-IDF v5.5.5 与 v6.0.2。
