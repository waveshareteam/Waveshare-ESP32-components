# BSP: Waveshare ESP32-P4-NANO

[![Component Registry](https://components.espressif.com/components/waveshare/esp32_p4_nano/badge.svg)](https://components.espressif.com/components/waveshare/esp32_p4_nano)

ESP32-P4-NANO is a small size and highly integrated development board designed by waveshare electronics based on ESP32-P4 chip
| HW version | BSP Version |
| :--------: | :---------: |
|    [V1.0](http://www.waveshare.com/wiki/ESP32-P4-NANO)    |      ^1     |

## Configuration

Configuration in `menuconfig`.

Selection LCD display `Board Support Package(ESP32-P4) --> Display --> Select LCD type`
- Waveshare 101M-8001280-IPS-CT-K Display (default only when ESP32-P4 minimum chip revision is v3.0 or later; ESP32-P4 rev 3.x only)
- Waveshare 10.1-DSI-TOUCH-A Display 
- Waveshare 8-DSI-TOUCH-A Display
- Waveshare 7-DSI-TOUCH-A Display
- Waveshare 5-DSI-Touch-A Display
- Waveshare 10.1-DSI-Touch-B Display
- Waveshare 9-DSI-Touch-B Display
- Waveshare 2.8inch DSI LCD Display (ESP32-P4 rev 3.x only)
- Waveshare 3.4inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 4inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 4inch DSI LCD Display (ESP32-P4 rev 3.x only)
- Waveshare 5inch DSI LCD (D) Display (ESP32-P4 rev 3.x only)
- Waveshare 6.25inch DSI LCD Display (ESP32-P4 rev 3.x only)
- Waveshare 5inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 7inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 7.9inch DSI LCD Display (ESP32-P4 rev 3.x only)
- Waveshare 7inch DSI LCD (E) Display (ESP32-P4 rev 3.x only)
- Waveshare 8inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 10.1inch DSI LCD (C) Display (ESP32-P4 rev 3.x only)
- Waveshare 8.8inch DSI LCD Display (ESP32-P4 rev 3.x only)
- Waveshare 11.9inch DSI LCD Display (ESP32-P4 rev 3.x only)

Selection color format `Board Support Package(ESP32-P4) --> Display --> Select LCD color format`
- RGB565 (default)
- RGB888

Change MIPI DSI lane bitrate `Board Support Package(ESP32-P4) --> Display --> MIPI DSI lane bitrate (Mbps)`
- 1500 (default)

## ESP32-P4 chip revision compatibility

The DSI-TOUCH-A/B displays shared with `esp32_p4_platform` support ESP32-P4 chip rev v1.3 and rev v3.x. The additional Raspberry Pi adapter displays in this BSP, and the 101M-8001280-IPS-CT-K display, require ESP32-P4 chip rev v3.0 or later.

When selecting an ESP32-P4 rev 3.x-only display:
- Use ESP32-P4 rev v3.0 or later hardware. Rev v1.3 hardware is not supported for these LCDs.
- Set the ESP-IDF minimum chip revision to v3.0 before building (`CONFIG_ESP_REV_MIN_FULL=300` or higher).
- The BSP emits a build error when a rev 3.x-only display is selected with a lower minimum revision, and `bsp_display_new*()` / `bsp_touch_new()` return `ESP_ERR_NOT_SUPPORTED` if the runtime eFuse chip revision is lower than v3.0.

## Display Page


### Recommended display screen

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                            | ESP32-P4 chip rev | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------|-------------------|--------|
| [10.1-DSI-TOUCH-A](https://www.waveshare.com/10.1-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1-dsi-touch-a-1.jpg">                | [~~waveshare/esp_lcd_jd9365_10_1~~](display/lcd/esp_lcd_jd9365_10_1)<br/>[waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365) | v1.3 / v3.x       | ✅      |
| [101M-8001280-IPS-CT-K](https://www.waveshare.com/101m-8001280-ips-ct-k.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/101m-8001280-ips-ct-k-1.jpg"> | [~~waveshare/esp_lcd_jd9365_10_1~~](display/lcd/esp_lcd_jd9365_10_1)<br/>[waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)      | v3.x only         | ✅      |
| [8-DSI-TOUCH-A](https://www.waveshare.com/8-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">                         | [~~waveshare/esp_lcd_jd9365_8~~](display/lcd/esp_lcd_ili9881c)<br/>[waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)            | v1.3 / v3.x       | ✅      |
| [7-DSI-TOUCH-A](https://www.waveshare.com/7-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/-/7-dsi-touch-a-1.jpg">                         | [waveshare/esp_lcd_ili9881c](display/lcd/esp_lcd_ili9881c)            | v1.3 / v3.x       | ✅      |
| [5-DSI-TOUCH-A](https://www.waveshare.com/5-dsi-touch-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/-/5-dsi-touch-a-1_1.jpg">                       | [waveshare/esp_lcd_hx8394](display/lcd/esp_lcd_hx8394)                | v1.3 / v3.x       | ✅      |
| [10.1-DSI-TOUCH-B](https://www.waveshare.com/10.1-dsi-touch-b.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">                 | [waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)                | v1.3 / v3.x       | ✅      |
| [9-DSI-TOUCH-B](https://www.waveshare.com/9-dsi-touch-b.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/-/8-dsi-touch-a-1.jpg">                       | [waveshare/esp_lcd_jd9365](display/lcd/esp_lcd_jd9365)                | v1.3 / v3.x       | ✅      |

### Common Raspberry adapter screen


<details open>
<summary>View full display</summary>

| Product ID                                                                                                                                                                                                                                                                                               | Dependency                                                       | ESP32-P4 chip rev | tested |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|-------------------|--------|
| [2.8inch DSI LCD](https://www.waveshare.com/2.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/thumbnail/122x122/9df78eab33525d08d6e5fb8d27136e95/2/_/2.8inch-dsi-lcd-3.jpg">               | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [3.4inch DSI LCD (C)](https://www.waveshare.com/3.4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/3/_/3.4inch-dsi-lcd-c-1.jpg">           | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [4inch DSI LCD (C)](https://www.waveshare.com/4inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-c-1.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [4inch DSI LCD](https://www.waveshare.com/4inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/4/i/4inch-dsi-lcd-1.jpg">                         | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [5inch DSI LCD (D)](https://www.waveshare.com/5inch-dsi-lcd-d.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-d-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [6.25inch DSI LCD](https://www.waveshare.com/6.25inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/6/_/6.25inch-dsi-lcd-2.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [5inch DSI LCD (C)](https://www.waveshare.com/5inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/5/i/5inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [7inch DSI LCD (C)](https://www.waveshare.com/7inch-dsi-lcd-c-with-case-a.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-c-4.jpg">     | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [7.9inch DSI LCD](https://www.waveshare.com/7.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/_/7.9inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [7inch DSI LCD (E)](https://www.waveshare.com/7inch-dsi-lcd-e.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/7/i/7inch-dsi-lcd-e-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [8inch DSI LCD (C)](https://www.waveshare.com/8inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/i/8inch-dsi-lcd-c-2.jpg">                 | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [10.1inch DSI LCD (C)](https://www.waveshare.com/10.1inch-dsi-lcd-c.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/0/10.1inch-dsi-lcd-c-2.jpg">        | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [8.8inch DSI LCD](https://www.waveshare.com/8.8inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/8/_/8.8inch-dsi-lcd-2.jpg">                   | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
| [11.9inch DSI LCD](https://www.waveshare.com/11.9inch-dsi-lcd.htm) <br/><img style="width: 150px; height: auto; display: block; margin: 0 auto;" src="https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/1/1/11.9inch-dsi-lcd-3.jpg">                | [waveshare/esp_lcd_dsi](display/lcd/esp_lcd_dsi)                 | v3.x only         | ✅      |
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
|  Capability |     Available    | Component                                                                                                | Version |
|-------------|------------------|----------------------------------------------------------------------------------------------------------|---------|
|   DISPLAY   |:heavy_check_mark:| [~~waveshare/esp_lcd_jd9365_10_1~~](https://components.espressif.com/components/waveshare/esp_lcd_jd9365_10_1) | 1.0.3   |
|   DISPLAY   |:heavy_check_mark:| [~~waveshare/esp_lcd_jd9365_8~~](https://components.espressif.com/components/waveshare/esp_lcd_jd9365_8) | 1.0.4   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_jd9365](https://components.espressif.com/components/waveshare/esp_lcd_jd9365)       | 1.0.3   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_ili9881c](https://components.espressif.com/components/waveshare/esp_lcd_ili9881c)     | 1.0.1   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_hx8394](https://components.espressif.com/components/waveshare/esp_lcd_hx8394)       | 1.0.2   |
|   DISPLAY   |:heavy_check_mark:| [waveshare/esp_lcd_dsi](https://components.espressif.com/components/waveshare/esp_lcd_dsi)               | 1.0.3   |
|  LVGL_PORT  |:heavy_check_mark:| [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)           | ^2      |
|    TOUCH    |:heavy_check_mark:| [espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911) | ^1      |
|   BUTTONS   |        :x:       |                                                                                                          |         |
|    AUDIO    |:heavy_check_mark:| [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)           | 1.2.*   |
|AUDIO_SPEAKER|:heavy_check_mark:|                                                                                                          |         |
|  AUDIO_MIC  |:heavy_check_mark:|                                                                                                          |         |
|    SDCARD   |:heavy_check_mark:| idf                                                                                                      | >=5.3   |
|     IMU     |        :x:       |                                                                                                          |         |
<!-- Autogenerated end: Dependencies -->
