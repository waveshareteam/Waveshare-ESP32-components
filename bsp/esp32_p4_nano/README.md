# BSP: Waveshare ESP32-P4-NANO

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_p4_nano/badge.svg)](https://components.espressif.com/components/waveshare/esp32_p4_nano)

ESP32-P4-NANO is a small size and highly integrated development board designed by waveshare electronics based on ESP32-P4 chip
| HW version | BSP Version |
| :--------: | :---------: |
|    [V1.0](http://www.waveshare.com/wiki/ESP32-P4-NANO)    |      ^1     |

## Configuration

Configuration in `menuconfig`.

Selection LCD display `Board Support Package(ESP32-P4) --> Display --> Select LCD type`
- Waveshare 101M-8001280-IPS-CT-K Display (default)
- Waveshare 10.1-DSI-TOUCH-A Display 
- Waveshare 8-DSI-TOUCH-A Display
- Waveshare 7-DSI-TOUCH-A Display
- Waveshare 2.8inch DSI LCD Display
- Waveshare 3.4inch DSI LCD (C) Display
- Waveshare 4inch DSI LCD (C) Display
- Waveshare 4inch DSI LCD Display
- Waveshare 5inch DSI LCD (D) Display
- Waveshare 6.25inch DSI LCD Display
- Waveshare 5inch DSI LCD (C) Display
- Waveshare 7inch DSI LCD (C) Display
- Waveshare 7.9inch DSI LCD Display
- Waveshare 7inch DSI LCD (E) Display
- Waveshare 8inch DSI LCD (C) Display
- Waveshare 10.1inch DSI LCD (C) Display
- Waveshare 8.8inch DSI LCD Display
- Waveshare 11.9inch DSI LCD Display

Selection color format `Board Support Package(ESP32-P4) --> Display --> Select LCD color format`
- RGB565 (default)
- RGB888

Change MIPI DSI lane bitrate `Board Support Package(ESP32-P4) --> Display --> MIPI DSI lane bitrate (Mbps)`
- 1500 (default)

## Display Page


### Recommended display screen

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                       | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|--------|
| [10.1-DSI-TOUCH-A](https://www.waveshare.com/10.1-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1-dsi-touch-a-1.jpg"> | [waveshare/esp_lcd_jd9365_10_1](display/lcd/esp_lcd_jd9365_10_1) | ✅      |
| [101M-8001280-IPS-CT-K](https://www.waveshare.com/101m-8001280-ips-ct-k.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/101m-8001280-ips-ct-k-1.jpg"> | [waveshare/esp_lcd_jd9365_10_1](display/lcd/esp_lcd_jd9365_10_1) | ✅      |
| [7-DSI-TOUCH-A](https://www.waveshare.com/7-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/-/7-dsi-touch-a-1.jpg"> | [waveshare/esp_lcd_ili9881c](display/lcd/esp_lcd_ili9881c)    | ✅      |

### Common Raspberry adapter screen


<details open>
<summary>View full display</summary>

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                       | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|--------|
| [2.8inch DSI LCD](https://www.waveshare.com/2.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/thumbnail/122x122/9df78eab33525d08d6e5fb8d27136e95/2/_/2.8inch-dsi-lcd-3.jpg">               | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [3.4inch DSI LCD (C)](https://www.waveshare.com/3.4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/3/_/3.4inch-dsi-lcd-c-1.jpg">           | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [4inch DSI LCD (C)](https://www.waveshare.com/4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-c-1.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [4inch DSI LCD](https://www.waveshare.com/4inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-1.jpg">                         | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [5inch DSI LCD (D)](https://www.waveshare.com/5inch-dsi-lcd-d.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-d-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [6.25inch DSI LCD](https://www.waveshare.com/6.25inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/6/_/6.25inch-dsi-lcd-2.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [5inch DSI LCD (C)](https://www.waveshare.com/5inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7inch DSI LCD (C)](https://www.waveshare.com/7inch-dsi-lcd-c-with-case-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-c-4.jpg">     | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7.9inch DSI LCD](https://www.waveshare.com/7.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/_/7.9inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [7inch DSI LCD (E)](https://www.waveshare.com/7inch-dsi-lcd-e.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-e-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [8inch DSI LCD (C)](https://www.waveshare.com/8inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/i/8inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [10.1inch DSI LCD (C)](https://www.waveshare.com/10.1inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1inch-dsi-lcd-c-2.jpg">        | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [8.8inch DSI LCD](https://www.waveshare.com/8.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/_/8.8inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
| [11.9inch DSI LCD](https://www.waveshare.com/11.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/1/11.9inch-dsi-lcd-3.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | ✅      |
</details>


## BackLight
```c
bsp_display_brightness_init();

bsp_display_backlight_on();

bsp_display_backlight_off();

bsp_display_brightness_set(100);
```

<!-- Autogenerated start: Dependencies -->
### Capabilities and dependencies
|  Capability |     Available    | Component                                                                                           | Version |
|-------------|------------------|-----------------------------------------------------------------------------------------------------|---------|
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_jd9365_10_1](https://components.espressif.com/components/waveshare/esp_lcd_jd9365_10_1) | 1.0.3   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_jd9365_8](https://components.espressif.com/components/waveshare/esp_lcd_jd9365_8) | 1.0.4   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_ili9881c](https://components.espressif.com/components/waveshare/esp_lcd_ili9881c) | 1.0.1   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_dsi](https://components.espressif.com/components/waveshare/esp_lcd_dsi)          | 1.0.3   |
|  LVGL_PORT  |:heavy_check_mark:| [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)      | ^2      |
|    TOUCH    |:heavy_check_mark:| [espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911) | ^1      |
|   BUTTONS   |        :x:       |                                                                                                     |         |
|    AUDIO    |:heavy_check_mark:| [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)      | 1.2.*   |
|AUDIO_SPEAKER|:heavy_check_mark:|                                                                                                     |         |
|  AUDIO_MIC  |:heavy_check_mark:|                                                                                                     |         |
|    SDCARD   |:heavy_check_mark:| idf                                                                                                 | >=5.3   |
|     IMU     |        :x:       |                                                                                                     |         |
<!-- Autogenerated end: Dependencies -->
