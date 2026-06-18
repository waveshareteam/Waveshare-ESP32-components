if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(REQ esp_driver_i2c)
else()
    set(REQ driver)
endif()

idf_component_register(SRCS "qmi8658.c" INCLUDE_DIRS "include" REQUIRES ${REQ})
