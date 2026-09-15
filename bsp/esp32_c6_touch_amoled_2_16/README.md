# BSP: Waveshare ESP32-C6-Touch-AMOLED-2.16

[简体中文](README_ZH.md)

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_c6_touch_amoled_2_16/badge.svg)](https://components.espressif.com/components/waveshare/esp32_c6_touch_amoled_2_16)

Reusable ESP-IDF board support for the Waveshare
[ESP32-C6-Touch-AMOLED-2.16](https://www.waveshare.com/esp32-c6-touch-amoled-2.16.htm).
The board uses an ESP32-C6, 480 x 480 SH8601 QSPI AMOLED, CST9217 touch,
AXP2101 PMU, QMI8658 IMU, PCF85063 RTC, ES8311 playback codec, ES7210 capture
codec and a microSD slot.

## Public API

Include the umbrella header:

~~~c
#include "bsp/esp-bsp.h"
~~~

Initialize the shared buses and board devices through the BSP:

~~~c
ESP_ERROR_CHECK(bsp_pmu_init());
i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();

lv_display_t *display = bsp_display_start();
assert(display != NULL);

ESP_ERROR_CHECK(bsp_sdcard_mount());
esp_codec_dev_handle_t speaker = bsp_audio_codec_speaker_init();
esp_codec_dev_handle_t microphone = bsp_audio_codec_microphone_init();
~~~

The BSP exposes the shared I2C0 handle and public addresses for AXP2101
(0x34), QMI8658 (0x6B) and PCF85063 (0x51). Applications should use the
managed QMI8658 and PCF85063 drivers rather than embedding board-specific bus
wrappers.

`bsp_pmu_get_snapshot()` reports battery presence, charge state and battery
voltage. AMOLED reset/power is controlled by AXP2101 ALDO3; audio amplifier
power is controlled by ALDO2. The board configuration programs ALDO4 to 1.8 V,
as specified by the product schematic.

## Shared SPI2 bus

The display uses GPIO0-4 plus CS GPIO15 in QSPI mode. The microSD slot shares
GPIO0 (clock), GPIO1 (MOSI) and GPIO2 (MISO), with CS GPIO6. The BSP initializes
one QSPI-capable SPI2 bus and adds both devices to that bus, so display and SD
can coexist without a second `spi_bus_initialize()` call.

The display, LVGL adapter, audio channels and shared SPI bus are singleton
resources intended to live for the application lifetime. LVGL calls from
application tasks must be protected with `bsp_display_lock()` and
`bsp_display_unlock()`.

## Dependency

Before Component Registry publication, pin the integration to the full commit
SHA from the BSP pull request:

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_2_16:
    git: https://github.com/waveshareteam/Waveshare-ESP32-components.git
    path: bsp/esp32_c6_touch_amoled_2_16
    version: "<full-component-commit-sha>"
~~~

After version 1.0.0 is published:

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_2_16: "^1.0.0"
~~~

## Component dependencies

Version 1.0.0 declares ESP-IDF >=5.5, LVGL >=8,<10,
`esp_codec_dev ~1.5`, `waveshare/esp_lcd_sh8601 ^2.0.0`,
`waveshare/esp_lcd_touch_cst9217 ^2.0.0` and
`espressif/esp_lvgl_adapter ~0.6`. The target is `esp32c6`; repository CI tests
ESP-IDF v5.5.5 and v6.0.2.

## Versioning

The initial reusable release is 1.0.0. Backward-compatible fixes increment the
patch version, additive public APIs increment the minor version, and breaking
API or pinout changes increment the major version.
