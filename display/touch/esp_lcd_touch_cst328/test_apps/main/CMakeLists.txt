if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(DRIVER_REQ esp_driver_i2c)
else()
    set(DRIVER_REQ driver)
endif()

idf_component_register(SRCS "test_esp_lcd_touch_cst328.c"
                       REQUIRES esp_lcd_touch_cst328 ${DRIVER_REQ} esp_timer unity)
