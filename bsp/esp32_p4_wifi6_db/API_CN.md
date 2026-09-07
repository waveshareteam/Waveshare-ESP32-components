# ESP32-P4-WIFI6-DB BSP API

[English Version](./API.md)

使用下面的方式包含完整的板级 API：

```c
#include "bsp/esp-bsp.h"
```

板级专用头文件为 `bsp/esp32_p4_wifi6_db.h`。

## Capabilities

| Capability | Macro | Value |
| --- | --- | ---: |
| 显示 | `BSP_CAPS_DISPLAY` | 1 |
| 触摸 | `BSP_CAPS_TOUCH` | 1 |
| 音频 | `BSP_CAPS_AUDIO` | 1 |
| 扬声器 | `BSP_CAPS_AUDIO_SPEAKER` | 1 |
| 麦克风 | `BSP_CAPS_AUDIO_MIC` | 1 |
| SD 卡 | `BSP_CAPS_SDCARD` | 1 |
| 摄像头 | `BSP_CAPS_CAMERA` | 1 |
| RTC | `BSP_CAPS_RTC` | 0 |

## 板级和 I2C

- `bsp_get_header_gpios()` 返回扩展排针的 GPIO 数组。
- `bsp_i2c_init()` 初始化 GPIO7/GPIO8。
- `bsp_i2c_get_handle()` 返回共享的主机总线句柄。
- `bsp_i2c_deinit()` 在没有设备继续占用时释放共享总线。

ES8311、GT911、显示背光控制器和摄像头 SCCB 控制器共享这条总线。

## ES8311 音频

- `bsp_audio_init()` 创建全双工 I2S 通道。
- `bsp_audio_codec_speaker_init()` 创建 ES8311 播放设备。
- `bsp_audio_codec_microphone_init()` 创建 ES8311 单麦克风采集设备。

I2S 引脚为 MCLK GPIO13、SCLK GPIO12、LRCK GPIO10、DSDIN GPIO9 和 ASDOUT GPIO11。

## 存储

- `bsp_sdcard_mount()` 和 `bsp_sdcard_unmount()` 使用本板的 4-bit SDMMC 接线。
- `bsp_sdcard_sdmmc_mount()` 接受调用者提供的挂载、host 或 slot 配置。
- `bsp_sdcard_get_sdmmc_host()` 用本板的 host 设置（包括 SD IO 供电句柄）填充
  `sdmmc_host_t`。
- `bsp_sdcard_sdmmc_get_slot()` 用本板的 4-bit slot 接线填充
  `sdmmc_slot_config_t`。
- `bsp_sdcard_get_handle()` 返回已挂载的卡。
- `bsp_spiffs_mount()` 和 `bsp_spiffs_unmount()` 提供可选的内部 SPIFFS 辅助函数。

## MIPI-DSI 显示

- `bsp_display_new()` 创建由 `BSP_LCD_TYPE` 选定的面板：JD9365 800x1280
  （8/10.1 英寸）、ILI9881C 720x1280（7 英寸）或 HX8394 720x1280（5 英寸）。
- `bsp_display_new_with_handles()` 同时返回 DSI 总线句柄。
- `bsp_display_delete()` 释放面板、DSI 总线、DSI PHY LDO 和背光设备。
- `bsp_display_get_panel_handle()` 返回 BSP 创建的面板句柄；显示启动前返回 NULL。
- `bsp_display_brightness_init()` 在共享 I2C 总线上注册背光控制器并将亮度置 0，
  `bsp_display_new()` 会调用它。
- `bsp_display_brightness_deinit()` 关闭背光并把设备从总线上移除。
- `bsp_display_brightness_set()` 向 I2C 地址 `0x45` 的寄存器 `0x96` 写入 8 位
  亮度值。
- `bsp_display_brightness_get()` 返回最后一次成功设置的百分比。
- `bsp_display_backlight_on()` 和 `bsp_display_backlight_off()` 分别设置 100%
  和 0%。

## GT911 触摸

- `bsp_touch_new()` 在共享的 GPIO7/GPIO8 I2C 总线上创建 GT911。
- 会依次尝试 GT911 的两个地址 `0x5D` 和 `0x14`。
- `BSP_LCD_TOUCH_RST` 和 `BSP_LCD_TOUCH_INT` 均为 `GPIO_NUM_NC`；触摸数据通过
  I2C 轮询读取。
- `bsp_touch_delete()` 释放触摸控制器及其 panel-I/O 句柄。

启用 LVGL 支持时：

- `bsp_display_start()` 初始化显示、GT911 和 LVGL 适配层。
- `bsp_display_start_with_config()` 接受旋转、防撕裂设置以及触摸坐标变换。
- `bsp_display_stop()` 停止 LVGL 并释放显示。
- `bsp_display_get_input_dev()` 返回已注册的 LVGL 触摸输入设备。
- `bsp_display_rotate()` 对显示应用 LVGL 旋转。
- `bsp_display_lock()` 和 `bsp_display_unlock()` 保护 LVGL 访问。
- `bsp_display_set_dummy_draw()`、`bsp_display_get_free_frame_buffer()` 和
  `bsp_display_flush_frame_buffer()` 支持直接操作帧缓冲的生产者。

## USB 和 CSI

- `bsp_usb_host_start()` 和 `bsp_usb_host_stop()` 管理 USB Host 库。
- `bsp_camera_start()` 初始化 MIPI-CSI 通路，并使用共享 I2C 总线作为摄像头 SCCB
  控制器。
