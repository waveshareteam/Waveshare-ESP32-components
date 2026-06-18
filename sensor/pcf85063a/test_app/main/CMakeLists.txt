if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(DRIVER_REQ esp_driver_gpio esp_driver_i2c)
else()
    set(DRIVER_REQ driver)
endif()

idf_component_register(SRCS "test_pcf85063a.c"
                       PRIV_REQUIRES spi_flash ${DRIVER_REQ}
                       INCLUDE_DIRS "")
