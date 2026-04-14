idf_component_register(
    SRCS "esp32_s3_lr1121_oled_1_54.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES esp_driver_i2c  esp_driver_gpio lvgl__lvgl espressif__esp_lvgl_adapter
    PRIV_REQUIRES esp_timer spiffs esp_psram fatfs
)

