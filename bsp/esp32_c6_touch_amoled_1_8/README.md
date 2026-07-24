# BSP: Waveshare ESP32-C6-Touch-AMOLED-1.8

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_c6_touch_amoled_1_8/badge.svg)](https://components.espressif.com/components/waveshare/esp32_c6_touch_amoled_1_8)

Reusable ESP-IDF board support for the Waveshare
[ESP32-C6-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-c6-touch-amoled-1.8.htm).
Both 368 x 448 hardware variants use the same ESP32-C6 pinout and are selected
at runtime without source changes.

## Hardware variants

| Variant | Display | Touch | Probe address | Panel offset |
| --- | --- | --- | --- | --- |
| V1 | SH8601 | FT3168/FT6146 through FT5x06 driver | 0x38 | none |
| V2 | CO5300 | CST820 through compatible CST816S driver | 0x15 | X gap 0x10 |

bsp_board_detect() resets the LCD and touch lines through TCA9554, probes
CST820 first and FT5x06 second, then caches the result. bsp_display_new(),
bsp_touch_new(), bsp_display_start() and bsp_display_start_with_config() all
use this detection path automatically.

## Public API

Include the umbrella header:

~~~c
#include "bsp/esp-bsp.h"
~~~

Board detection:

~~~c
typedef enum {
    BSP_BOARD_VARIANT_UNKNOWN = 0,
    BSP_BOARD_VARIANT_SH8601_FT5X06,
    BSP_BOARD_VARIANT_CO5300_CST820,
} bsp_board_variant_t;

bsp_board_variant_t bsp_board_detect(void);
bsp_board_variant_t bsp_board_get_variant(void);
const char *bsp_board_variant_to_name(bsp_board_variant_t variant);
~~~

Display and touch:

~~~c
esp_err_t bsp_display_new(const bsp_display_config_t *config,
                          esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io);
esp_err_t bsp_touch_new(const bsp_touch_config_t *config,
                        esp_lcd_touch_handle_t *ret_touch);

lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
~~~

Shared buses and board peripherals:

~~~c
esp_err_t bsp_i2c_init(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);
esp_io_expander_handle_t bsp_io_expander_init(void);

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);
esp_err_t bsp_audio_poweramp_enable(bool enable);

esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);
~~~

AXP2101 (0x34), QMI8658 (0x6B) and PCF85063 (0x51) are exposed through the
shared I2C0 handle and public address constants. Applications can add their
preferred managed drivers without duplicating the board bus configuration.

The QSPI display and SDSPI card slot both use the ESP32-C6 SPI2 peripheral on
different pins. They are supported in separate flows and cannot be active at
the same time; the BSP returns ESP_ERR_INVALID_STATE for the second user.

## Dependency

After publication to the Espressif Component Registry:

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_1_8: "^1.0.0"
~~~

Before registry publication, use a pushed branch, tag or commit ref:

~~~yaml
dependencies:
  waveshare/esp32_c6_touch_amoled_1_8:
    git: https://github.com/waveshareteam/Waveshare-ESP32-components.git
    path: bsp/esp32_c6_touch_amoled_1_8
    version: "master"
~~~

For reproducible integration, replace master with a full commit SHA. In a Git
dependency, version is a Git ref; it is not the component manifest version.

## Component dependencies

Version 1.0.0 declares:

~~~yaml
esp_codec_dev: "~1.5"
waveshare/esp_lcd_sh8601: "^2.0.0"
espressif/esp_lcd_co5300: "^2.0.3"
espressif/esp_lcd_touch: "^1.2.0"
espressif/esp_lcd_touch_ft5x06: "^1.1.0~1"
espressif/esp_lcd_touch_cst816s: "^1.1.1~1"
espressif/esp_lvgl_port: "^2"
lvgl/lvgl: ">=8,<10"
esp_lcd_panel_io_additions: "^1"
esp_io_expander_tca9554: "^2"
idf: ">=5.5"
~~~

The target is esp32c6. Repository CI is pinned to ESP-IDF v5.5.5 and v6.0.2.

## Versioning

The initial reusable release is 1.0.0. Backward-compatible fixes increment the
patch version, additive public APIs or new compatible hardware support
increment the minor version, and breaking API, pinout or board-detection
contract changes increment the major version.
