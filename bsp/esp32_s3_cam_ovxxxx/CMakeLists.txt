idf_component_register(
    SRCS "esp32_s3_cam_ovxxxx.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES fatfs esp_driver_i2c esp_driver_gpio esp_lcd esp32-camera custom_io_expander_ch32v003
    PRIV_REQUIRES esp_timer spiffs esp_psram usb
)