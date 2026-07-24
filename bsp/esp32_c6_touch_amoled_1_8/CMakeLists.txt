idf_component_register(
    SRCS "esp32_c6_touch_amoled_1_8.c"
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES
        esp_codec_dev
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_sdspi
        esp_io_expander_tca9554
        esp_lcd
        espressif__esp_lcd_touch
        espressif__esp_lvgl_port
        lvgl__lvgl
        sdmmc
    PRIV_REQUIRES
        esp_driver_spi
        esp_lcd_panel_io_additions
        esp_timer
        fatfs
        spiffs
)
