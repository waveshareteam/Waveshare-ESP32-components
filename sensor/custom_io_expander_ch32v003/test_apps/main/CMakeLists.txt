if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(DRIVER_REQ esp_driver_i2c)
else()
    set(DRIVER_REQ driver)
endif()

idf_component_register(
    SRCS "test_app_ch32v003.c"
    REQUIRES custom_io_expander_ch32v003 ${DRIVER_REQ} unity
    )
