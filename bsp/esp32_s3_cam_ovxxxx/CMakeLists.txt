if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(REQ esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_driver_sdmmc esp_driver_sdspi)
    set(PRIV_REQ esp_driver_spi)
else()
    set(REQ driver)
    set(PRIV_REQ "")
endif()

if(${IDF_VERSION_MAJOR} LESS 6)
    list(APPEND PRIV_REQ usb)
endif()

idf_component_register(
    SRCS "esp32_s3_cam_ovxxxx.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} fatfs esp_lcd esp32-camera custom_io_expander_ch32v003
    PRIV_REQUIRES ${PRIV_REQ} esp_timer spiffs esp_psram
)
