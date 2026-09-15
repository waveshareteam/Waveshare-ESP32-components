#ifndef TEST_STUB_DRIVER_I2C_MASTER_H
#define TEST_STUB_DRIVER_I2C_MASTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t unused;
} i2c_master_bus_t;

typedef struct {
    uint8_t unused;
} i2c_master_dev_t;

typedef i2c_master_bus_t *i2c_master_bus_handle_t;
typedef i2c_master_dev_t *i2c_master_dev_handle_t;

typedef enum {
    I2C_ADDR_BIT_LEN_7 = 0,
} i2c_addr_bit_len_t;

typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;
    struct {
        unsigned int disable_ack_check : 1;
    } flags;
} i2c_device_config_t;

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle,
                                    const i2c_device_config_t *dev_config,
                                    i2c_master_dev_handle_t *dev_handle);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev_handle,
                              const uint8_t *write_buffer,
                              size_t write_size,
                              int timeout_ms);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev_handle,
                                      const uint8_t *write_buffer,
                                      size_t write_size,
                                      uint8_t *read_buffer,
                                      size_t read_size,
                                      int timeout_ms);

#endif
