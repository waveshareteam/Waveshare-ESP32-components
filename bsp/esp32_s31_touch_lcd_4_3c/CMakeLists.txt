set(SRC
    src/bsp_audio.c
    src/bsp_bq27220.cpp
    src/bsp_board.c
    src/bsp_display.c
    src/bsp_i2c.c
    src/bsp_imu.c
    src/bsp_rtc.c
    src/bsp_storage.c
)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(REQ
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_ledc
        esp_driver_sdmmc
        esp_driver_sdspi
        fatfs
        spiffs
        vfs
    )
else()
    set(REQ driver fatfs spiffs vfs)
endif()

idf_component_register(
    SRCS ${SRC}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${REQ} esp_lcd
    PRIV_REQUIRES esp_psram esp_timer lewisxhe__sensorlib
)
