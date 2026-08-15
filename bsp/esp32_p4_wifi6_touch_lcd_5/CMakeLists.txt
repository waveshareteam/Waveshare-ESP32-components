
idf_component_register(
    SRCS "esp32_p4_wifi6_touch_lcd_5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_sdmmc esp_driver_i2s
    PRIV_REQUIRES esp_lcd usb spiffs fatfs esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_sdmmc esp_driver_i2s
)
