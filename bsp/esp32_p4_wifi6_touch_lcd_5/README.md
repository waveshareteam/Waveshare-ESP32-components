# BSP: Waveshare ESP32-P4-WIFI6-Touch-LCD-5

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_p4_wifi6_touch_lcd_5/badge.svg)](https://components.espressif.com/components/waveshare/esp32_p4_wifi6_touch_lcd_5)


| HW version | BSP Version |
| :--------: | :---------: |
|    [V1.0](http://www.waveshare.com/wiki/ESP32-P4-WIFI6-Touch-LCD-5)    |      ^1     |

## HX8394 initialization

This BSP selects `ESP_LCD_HX8394_SKIP_I2C_INIT` through its hidden
`BSP_LCD_HX8394_SKIP_I2C_INIT` bridge. The board BSP owns board-level I2C and
power behavior, while the standalone HX8394 driver retains its default I2C
sequence for other users.


## BackLight
```c
bsp_display_brightness_init();

bsp_display_backlight_on();

bsp_display_backlight_off();

bsp_display_brightness_set(100);
```
