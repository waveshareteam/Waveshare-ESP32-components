file(GLOB_RECURSE SRCS *.c)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_lcd
        espressif__esp_lvgl_port
        espressif__led_strip
        lvgl__lvgl
        sdmmc
    PRIV_REQUIRES
        esp_driver_ledc
        esp_driver_sdspi
        esp_driver_spi
        esp_psram
        esp_timer
        fatfs
        spiffs
)
