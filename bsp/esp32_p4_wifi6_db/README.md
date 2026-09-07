# BSP: ESP32-P4-WIFI6-DB

[中文版本](./README_CN.md)

## Overview

This component provides the board support package for ESP32-P4-WIFI6-DB.
The current manifest version is `0.0.1`.

Supported board interfaces:

- SD card over 4-bit SDMMC.
- ES8311 playback and one analog microphone input.
- Shared I2C bus.
- Selectable two-lane MIPI-DSI display using JD9365, ILI9881C, or HX8394.
- GT911 capacitive touch controller.
- MIPI-CSI camera through `esp_video`.
- USB Host.
- Expansion-header GPIO enumeration.

The board does not provide an external RTC or an LCD reset GPIO. The audio
power amplifier is enabled by GPIO53. The GT911 reset and interrupt signals
are not connected.

## Pin Assignment

### I2C

| Signal | GPIO |
| --- | ---: |
| SDA | 7 |
| SCL | 8 |

The ES8311, GT911, LCD backlight controller, and camera SCCB bus share this I2C
bus.

### ES8311

| ES8311 signal | ESP32-P4 signal | GPIO |
| --- | --- | ---: |
| MCLK | I2S MCLK | 13 |
| SCLK | I2S BCLK | 12 |
| LRCK | I2S WS | 10 |
| DSDIN | I2S DOUT | 9 |
| ASDOUT | I2S DIN | 11 |
| Power amplifier enable | GPIO | 53 |

`bsp_audio_codec_speaker_init()` and
`bsp_audio_codec_microphone_init()` both create ES8311 codec devices. The
microphone path is configured for one analog microphone.

### SD Card

| Signal | GPIO |
| --- | ---: |
| D0 | 39 |
| D1 | 40 |
| D2 | 41 |
| D3 | 42 |
| CMD | 44 |
| CLK | 43 |
| Power enable | 45, active low |

`bsp_sdcard_mount()` disables the card power for 100 ms, re-enables it, and
waits another 200 ms before starting SDMMC initialization. On ESP32-P4, SDMMC
IO voltage is controlled through on-chip LDO channel 4. The BSP also enables
the SDMMC internal pull-ups; external pull-ups are still required for normal
signal integrity.

### Display

Select the connected panel in `menuconfig`. The 10.1-inch JD9365 panel is the
default.

| Panel option | Controller | Resolution | DSI lane bit rate |
| --- | --- | ---: | ---: |
| Waveshare 5-DSI-TOUCH-A | HX8394 | 720x1280 | 700 Mbps |
| Waveshare 7-DSI-TOUCH-A | ILI9881C | 720x1280 | 1000 Mbps |
| Waveshare 8-DSI-TOUCH-A | JD9365 | 800x1280 | 1500 Mbps |
| Waveshare 10.1-DSI-TOUCH-A | JD9365 | 800x1280 | 1500 Mbps |

All four options use two DSI data lanes. The LCD reset pin is `GPIO_NUM_NC`.

Backlight brightness is controlled on the shared I2C bus:

| Item | Value |
| --- | --- |
| I2C address | `0x45` |
| Brightness register | `0x96` |
| Brightness data | `0..255`, scaled from `0..100%` |

`bsp_display_brightness_set()` sends `{0x96, data}` where
`data = 255 * brightness_percent / 100`.

### Touch

The GT911 uses the shared I2C bus on GPIO7/GPIO8. Its reset and interrupt GPIOs
are both `GPIO_NUM_NC`, so the driver polls the controller. `bsp_touch_new()`
tries both valid GT911 addresses, `0x5D` and `0x14`.

`bsp_display_start()` and `bsp_display_start_with_config()` automatically
create the GT911 device and register it with the LVGL adapter.

### Expansion Header

`bsp_get_header_gpios()` returns the following BSP-owned, read-only array:

```text
52, 51, 31, 30, 29, 28, 50, 49, 5, 4, 3, 2, 24,
25, 20, 21, 22, 23, 26, 27, 32, 33, 46, 47, 48
```

## Configuration

Use `menuconfig` to select:

- I2C controller and 100/400 kHz bus speed.
- I2S controller.
- Connected MIPI-DSI panel type.
- RGB565 or RGB888 display format.
- One to three MIPI-DPI frame buffers.
- SD card and SPIFFS mount options.

## Basic Usage

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

The current BSP changes were flashed and verified with ESP-IDF v6.0.1. That
verification does not replace feature-specific checks for display output, audio
playback/capture, SD card operation, USB, CSI, touch input, or electrical
behavior.
