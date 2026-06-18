if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(REQ esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_driver_sdmmc esp_driver_spi esp_driver_tsens)
else()
    set(REQ driver)
endif()

idf_component_register(
    SRCS "esp32_s3_lr1121_oled_1_54.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} lvgl__lvgl espressif__esp_lvgl_adapter
    PRIV_REQUIRES esp_timer spiffs esp_psram fatfs
)

