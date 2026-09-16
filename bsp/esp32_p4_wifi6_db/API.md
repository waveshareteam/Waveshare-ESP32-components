# ESP32-P4-WIFI6-DB BSP API

[中文版本](./API_CN.md)

Include the complete board API with:

```c
#include "bsp/esp-bsp.h"
```

The board-specific header is `bsp/esp32_p4_wifi6_db.h`.

## Capabilities

| Capability | Macro | Value |
| --- | --- | ---: |
| Display | `BSP_CAPS_DISPLAY` | 1 |
| Touch | `BSP_CAPS_TOUCH` | 1 |
| Audio | `BSP_CAPS_AUDIO` | 1 |
| Speaker | `BSP_CAPS_AUDIO_SPEAKER` | 1 |
| Microphone | `BSP_CAPS_AUDIO_MIC` | 1 |
| SD card | `BSP_CAPS_SDCARD` | 1 |
| Camera | `BSP_CAPS_CAMERA` | 1 |
| RTC | `BSP_CAPS_RTC` | 0 |

## Board and I2C

- `bsp_get_header_gpios()` returns the expansion-header GPIO array.
- `bsp_i2c_init()` initializes GPIO7/GPIO8.
- `bsp_i2c_get_handle()` returns the shared master-bus handle.
- `bsp_i2c_deinit()` releases the shared bus when no device still owns it.

The ES8311, GT911, display backlight controller, and camera SCCB controller
share this bus.

## ES8311 Audio

- `bsp_audio_init()` creates full-duplex I2S channels.
- `bsp_audio_codec_speaker_init()` creates the ES8311 playback device.
- `bsp_audio_codec_microphone_init()` creates the ES8311 single-microphone
  capture device.

I2S pins are MCLK GPIO13, SCLK GPIO12, LRCK GPIO10, DSDIN GPIO9, and ASDOUT
GPIO11.

## Storage

- `bsp_sdcard_mount()` and `bsp_sdcard_unmount()` use the board's 4-bit SDMMC
  wiring.
- `bsp_sdcard_sdmmc_mount()` accepts caller-provided mount, host, or slot
  configuration.
- `bsp_sdcard_get_sdmmc_host()` fills an `sdmmc_host_t` with the board's host
  settings, including the SD IO power handle.
- `bsp_sdcard_sdmmc_get_slot()` fills an `sdmmc_slot_config_t` with the board's
  4-bit slot wiring.
- `bsp_sdcard_get_handle()` returns the mounted card.
- `bsp_spiffs_mount()` and `bsp_spiffs_unmount()` provide the optional
  internal SPIFFS helper.

## MIPI-DSI Display

- `bsp_display_new()` creates the panel selected by `BSP_LCD_TYPE`: JD9365
  800x1280 (8/10.1 inch), ILI9881C 720x1280 (7 inch), or HX8394 720x1280
  (5 inch).
- `bsp_display_new_with_handles()` also returns the DSI bus handle.
- `bsp_display_delete()` releases the panel, DSI bus, DSI PHY LDO, and
  backlight device.
- `bsp_display_get_panel_handle()` returns the panel created by the BSP, or
  NULL before the display is started.
- `bsp_display_brightness_init()` registers the backlight controller on the
  shared I2C bus and sets brightness to 0. `bsp_display_new()` calls it.
- `bsp_display_brightness_deinit()` blanks the backlight and removes it from
  the bus.
- `bsp_display_brightness_set()` writes an 8-bit brightness value to I2C
  address `0x45`, register `0x96`.
- `bsp_display_brightness_get()` returns the last successfully configured
  percentage.
- `bsp_display_backlight_on()` and `bsp_display_backlight_off()` set 100% and
  0%.

## GT911 Touch

- `bsp_touch_new()` creates the GT911 on the shared GPIO7/GPIO8 I2C bus.
- Both GT911 addresses, `0x5D` and `0x14`, are tried.
- `BSP_LCD_TOUCH_RST` and `BSP_LCD_TOUCH_INT` are `GPIO_NUM_NC`; touch data is
  polled over I2C.
- `bsp_touch_delete()` releases the touch controller and its panel-I/O handle.

When LVGL support is enabled:

- `bsp_display_start()` initializes the display, GT911, and LVGL adapter.
- `bsp_display_start_with_config()` accepts rotation and tear-avoidance
  settings plus touch coordinate transforms.
- `bsp_display_stop()` stops LVGL and releases the display.
- `bsp_display_get_input_dev()` returns the registered LVGL touch input device.
- `bsp_display_rotate()` applies an LVGL rotation to a display.
- `bsp_display_lock()` and `bsp_display_unlock()` guard LVGL access.
- `bsp_display_set_dummy_draw()`,
  `bsp_display_get_free_frame_buffer()`, and
  `bsp_display_flush_frame_buffer()` support direct framebuffer producers.

## USB and CSI

- `bsp_usb_host_start()` and `bsp_usb_host_stop()` manage the USB Host
  library.
- `bsp_camera_start()` initializes the MIPI-CSI path with the shared I2C bus
  as the camera SCCB controller.
