if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(DRIVER_REQ esp_driver_gpio esp_driver_spi)
else()
    set(DRIVER_REQ driver)
endif()

idf_component_register(SRCS "test_esp_lcd_st7735.c"
                       REQUIRES esp_lcd_st7735 esp_lcd ${DRIVER_REQ} esp_timer unity)
