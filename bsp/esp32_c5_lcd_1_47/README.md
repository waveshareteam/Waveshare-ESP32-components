# BSP: ESP32-C5-LCD-1.47

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_c5_lcd_1_47/badge.svg)](https://components.espressif.com/components/waveshare/esp32_c5_lcd_1_47)
![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg)

## Overview

ESP32-C5-LCD-1.47 is a Waveshare ESP32-C5 board with a 1.47-inch SPI LCD. This BSP provides board-level initialization helpers for the display, storage, LVGL, and onboard WS2812 LED.

Main features:

- ESP32-C5 target support
- 1.47-inch ST7789 SPI LCD, 172 x 320, RGB565
- LVGL port integration with configurable draw buffer height
- PWM LCD backlight brightness control
- SPIFFS virtual filesystem helpers
- SDSPI microSD card mount and unmount helpers
- Onboard WS2812 LED strip helper APIs

## Pin Assignment

| Function | GPIO |
| --- | --- |
| LCD CS | GPIO23 |
| LCD CLK | GPIO7 |
| LCD MOSI | GPIO6 |
| LCD DC | GPIO24 |
| LCD RST | GPIO26 |
| LCD Backlight | GPIO10 |
| microSD D0/MISO | GPIO5 |
| microSD CMD/MOSI | GPIO6 |
| microSD CLK | GPIO7 |
| microSD CS | GPIO4 |
| WS2812 data | GPIO8 |

## Capabilities and Dependencies

| Available | Capability | Component | Version |
| --- | --- | --- | --- |
| Yes | Display | IDF `esp_lcd` | `>=5.3` |
| Yes | LVGL port | `espressif/esp_lvgl_port` | `^2` |
| Yes | LVGL | `lvgl/lvgl` | `>=8,<10` |
| Yes | WS2812 LED | `espressif/led_strip` | `*` |
| Yes | SPIFFS | IDF `spiffs` | `>=5.3` |
| Yes | microSD card | IDF `sdmmc`, `fatfs` | `>=5.3` |
| No | Touch | - | - |
| No | Buttons | - | - |
| No | Audio | - | - |
| No | IMU | - | - |

## Configuration

The following options are available in `menuconfig` under `Board Support Package`:

- `BSP_ERROR_CHECK`: Enable assert-style error checking in BSP helpers.
- `BSP_I2C_NUM`: Select the I2C peripheral used by BSP I2C helpers.
- `BSP_I2C_FAST_MODE`: Select 400 kHz I2C mode instead of 100 kHz mode.
- `BSP_SPIFFS_*`: Configure SPIFFS mount behavior, partition label, mount point, and max open files.
- `BSP_SD_*`: Configure microSD card mount point and format-on-failure behavior.
- `BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT`: Configure the display draw buffer height used by the BSP.
- `BSP_DISPLAY_BRIGHTNESS_LEDC_CH`: Select the LEDC channel used for LCD backlight PWM.
- `BSP_DISPLAY_LVGL_BUF_HEIGHT`: Configure the LVGL draw buffer height.

## Basic Usage

```c
#include "bsp/esp-bsp.h"

void app_main(void)
{
    lv_display_t *display = bsp_display_start();
    if (display) {
        bsp_display_backlight_on();
        bsp_display_brightness_set(80);
    }

    bsp_ws2812b_init();
    bsp_setledcolor(0, 255, 0, 0);
}
```

## Storage Usage

```c
#include "bsp/esp-bsp.h"

void app_main(void)
{
    bsp_spiffs_mount();
    bsp_sdcard_mount();
}
```
