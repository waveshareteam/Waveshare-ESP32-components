if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(SRC_VER "esp32_s3_touch_amoled_1_8_bsp_idf4.c")
else()
    set(SRC_VER "esp32_s3_touch_amoled_1_8_bsp_idf5.c")
endif()

idf_component_register(
    SRCS "esp32_s3_touch_amoled_1_8.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs
)
