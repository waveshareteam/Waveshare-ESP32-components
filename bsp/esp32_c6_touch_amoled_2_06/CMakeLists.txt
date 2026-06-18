if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(REQ esp_driver_gpio esp_driver_i2c esp_driver_i2s)
    set(PRIV_REQ esp_driver_ledc)
else()
    set(REQ driver)
    set(PRIV_REQ "")
endif()

idf_component_register(
    SRCS "esp32_c6_touch_amoled_2_06.c" ${SRC_VER}
    INCLUDE_DIRS "include" "include/bsp"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} esp_lcd
    PRIV_REQUIRES ${PRIV_REQ} esp_timer spiffs esp_psram fatfs
)
