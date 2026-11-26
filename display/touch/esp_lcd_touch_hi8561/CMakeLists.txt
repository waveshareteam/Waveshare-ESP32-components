if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(PUB_I2C_REQ esp_driver_i2c)
    set(PRIV_REQ esp_driver_gpio)
else()
    set(PUB_I2C_REQ driver)
    set(PRIV_REQ "")
endif()

idf_component_register(SRCS "esp_lcd_touch_hi8561.c" INCLUDE_DIRS "include" REQUIRES "esp_lcd" "esp_lcd_touch" ${PUB_I2C_REQ} PRIV_REQUIRES ${PRIV_REQ})
