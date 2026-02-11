
idf_component_register(
    SRCS "esp32_p4_wifi6_touch_lcd_x.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs
)
