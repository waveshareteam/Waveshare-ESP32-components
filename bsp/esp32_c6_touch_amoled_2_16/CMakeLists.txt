idf_component_register(
    SRCS "esp32_c6_touch_amoled_2_16.c"
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES
        esp_codec_dev
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_sdspi
        esp_driver_spi
        esp_lcd
        espressif__esp_lcd_touch
        espressif__esp_lvgl_adapter
        lvgl__lvgl
        sdmmc
        waveshare__esp_lcd_sh8601
        waveshare__esp_lcd_touch_cst9217
    PRIV_REQUIRES
        esp_timer
        fatfs
        spiffs
)
