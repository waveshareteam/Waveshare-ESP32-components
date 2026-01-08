# BSP: Waveshare ESP32-P4-PLATFORM

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_p4_platform/badge.svg)](https://components.espressif.com/components/waveshare/esp32_p4_platform)

This is a component library specifically designed for the Waveshare ESP32-P4 series development board, aimed at addressing display screen driving issues.
| BSP Version |
| :---------: |
|      ^1     |

## Configuration

Configuration in `menuconfig`.

Selection LCD display `Board Support Package(ESP32-P4) --> Display --> Select LCD type`

- Waveshare 3.4-DSI-Touch-C Display
- Waveshare 4-DSI-Touch-C Display
- Waveshare 5-DSI-Touch-A Display
- Waveshare 7-DSI-TOUCH-A Display
- Waveshare 7-DSI-TOUCH-C Display
- Waveshare 8-DSI-TOUCH-A Display
- Waveshare 9-DSI-Touch-B Display
- Waveshare 10.1-DSI-TOUCH-A Display (default)
- Waveshare 10.1-DSI-TOUCH-B Display

Selection color format `Board Support Package(ESP32-P4) --> Display --> Select LCD color format`
- RGB565 (default)
- RGB888

## Display Page


### Recommended display screen

| Product ID                                                                                                                                                                                                                                                                                | Dependency                                                                                                                      | tested |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|--------|
| [3.4-DSI-TOUCH-C](https://www.waveshare.com/4-dsi-touch-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/3/_/3.4-dsi-touch-c-1.jpg">      | [waveshare/esp_lcd_hx8394](display/lcd/esp_lcd_jd9365)                                                                          | ✅      |
| [4-DSI-TOUCH-C](https://www.waveshare.com/4-dsi-touch-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/-/4-dsi-touch-c-1.jpg">          | [waveshare/esp_lcd_hx8394](display/lcd/esp_lcd_jd9365)                                                                          | ✅      |
| [5-DSI-TOUCH-A](https://www.waveshare.com/5-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/-/5-dsi-touch-a-1_1.jpg">        | [waveshare/esp_lcd_hx8394](display/lcd/esp_lcd_hx8394)                                                                          | ✅      |
| [7-DSI-TOUCH-A](https://www.waveshare.com/7-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/-/7-dsi-touch-a-1_1.jpg">          | [waveshare/esp_lcd_ili9881c](display/lcd/esp_lcd_ili9881c)                                                                      | ✅      |
| [7-DSI-TOUCH-C](https://www.waveshare.com/7-dsi-touch-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/-/7-dsi-touch-c-1.jpg">        | [espressif/esp_lcd_ek79007](https://components.espressif.com/components/espressif/esp_lcd_ek79007)                                                                         | ✅      |
| [8-DSI-TOUCH-A](https://www.waveshare.com/8-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">          | [~~waveshare/esp_lcd_jd9365_8~~](display/lcd/esp_lcd_ili9881c)<br/>[waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)       | ✅      |
| [9-DSI-TOUCH-B](https://www.waveshare.com/9-dsi-touch-b.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">          | [waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)                                                                          | ✅      |
| [10.1-DSI-TOUCH-A](https://www.waveshare.com/10.1-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1-dsi-touch-a-1.jpg"> | [~~waveshare/esp_lcd_jd9365_10_1~~](display/lcd/esp_lcd_jd9365_10_1)<br/>[waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365) | ✅      |
| [10.1-DSI-TOUCH-B](https://www.waveshare.com/10.1-dsi-touch-b.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">    | [waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)                                                                          | ✅      |


## BackLight
```c
bsp_display_brightness_init();

bsp_display_backlight_on();

bsp_display_backlight_off();

bsp_display_brightness_set(100);
```

<!-- Autogenerated start: Dependencies -->
### Capabilities and dependencies
|  Capability |     Available    | Component                                                                                                  | Version |
|-------------|------------------|------------------------------------------------------------------------------------------------------------|---------|
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_jd9365](https://components.espressif.com/components/waveshare/esp_lcd_jd9365)           | 1.0.3   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_ili9881c](https://components.espressif.com/components/waveshare/esp_lcd_ili9881c)       | 1.0.1   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_hx8394](https://components.espressif.com/components/waveshare/esp_lcd_hx8394)           | 1.0.2   |
|   DISPLAY   |:heavy_check_mark:| [espressif/esp_lcd_ek79007](https://components.espressif.com/components/espressif/esp_lcd_ek79007)             | ^1    |
|  LVGL_PORT  |:heavy_check_mark:| [espressif/esp_lvgl_adapter](https://components.espressif.com/components/espressif/esp_lvgl_adapter)       | ^1      |
|    TOUCH    |:heavy_check_mark:| [espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911) | ^1      |
|   BUTTONS   |        :x:       |                                                                                                            |         |
|    AUDIO    |:heavy_check_mark:| [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)             | ^1.5    |
|AUDIO_SPEAKER|:heavy_check_mark:|                                                                                                            |         |
|  AUDIO_MIC  |:heavy_check_mark:|                                                                                                            |         |
|    SDCARD   |:heavy_check_mark:| idf                                                                                                        | >=5.5   |
|     IMU     |        :x:       |                                                                                                            |         |
<!-- Autogenerated end: Dependencies -->
