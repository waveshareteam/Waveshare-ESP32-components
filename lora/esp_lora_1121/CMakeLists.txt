file(GLOB_RECURSE SRCS  src/*.c
                        src/lr1121_printers/*.c
                        src/lr1121_modem/*.c 
                        src/lr1121_common/*.c
                        src/lr11xx_driver/*.c
)

set(INCLUDE_DIRS 
   include 
   include/lr1121_printers
   include/lr1121_modem
   include/lr1121_common
   include/lr11xx_driver
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_timer esp_driver_i2c esp_driver_spi esp_driver_gpio
)
