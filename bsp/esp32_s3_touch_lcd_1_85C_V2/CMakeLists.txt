idf_component_register(
    SRCS "esp32_s3_touch_lcd_1_85C_V2.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES fatfs esp_driver_i2c esp_driver_gpio esp_lcd pcf85063a esp_io_expander_tca9554
    PRIV_REQUIRES esp_timer spiffs esp_psram
)