if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(PRIV_REQ esp_driver_gpio)
else()
    set(PRIV_REQ driver)
endif()

idf_component_register(SRCS "esp_lcd_jd9365_8.c" INCLUDE_DIRS "include" PRIV_REQUIRES ${PRIV_REQ} REQUIRES "esp_lcd")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
