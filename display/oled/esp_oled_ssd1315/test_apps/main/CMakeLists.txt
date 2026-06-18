if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(DRIVER_REQ esp_driver_i2c)
else()
    set(DRIVER_REQ driver)
endif()

idf_component_register(SRCS "test_esp_oled_ssd1315.c"
                       REQUIRES esp_oled_ssd1315 esp_lcd ${DRIVER_REQ} esp_timer unity)
