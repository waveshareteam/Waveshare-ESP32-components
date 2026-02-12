# BSP: Waveshare ESP32-P4-WIFI6-Touch-LCD-XC

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_p4_wifi6_touch_lcd_xc/badge.svg)](https://components.espressif.com/components/waveshare/esp32_p4_wifi6_touch_lcd_xc)

ESP32-P4-NANO is a small size and highly integrated development board designed by waveshare electronics based on ESP32-P4 chip
| HW version | BSP Version |
| :--------: | :---------: |
|    [V1.0](http://www.waveshare.com/wiki/ESP32-P4-WIFI6-Touch-LCD-XC)    |      ^2     |

## Configuration

Configuration in `menuconfig`.

Selection LCD display `Board Support Package(ESP32-P4) --> Display --> Select LCD type`
- Waveshare board with 800*800 3.4-inch Display (default)
- Waveshare board with 720*720 4-inch Display


## BackLight
```c
bsp_display_brightness_init();

bsp_display_backlight_on();

bsp_display_backlight_off();

bsp_display_brightness_set(100);
```