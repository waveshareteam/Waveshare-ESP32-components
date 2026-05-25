/*
 * SPDX-FileCopyrightText: 2022-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LCD touch: HI8561
 */

#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a new HI8561 touch driver
 *
 * @note The I2C communication should be initialized before use this function.
 *
 * @param io LCD/Touch panel IO handle
 * @param config: Touch configuration
 * @param out_touch: Touch instance handle
 * @return
 *      - ESP_OK                    on success
 *      - ESP_ERR_NO_MEM            if there is no memory for allocating main structure
 */
esp_err_t esp_lcd_touch_new_i2c_hi8561(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config,
                                      esp_lcd_touch_handle_t *out_touch);

/**
 * @brief HI8561 Configuration Type
 *
 */
typedef struct {
    uint8_t dev_addr;  /*!< I2C device address */
    uint32_t touch_info_start_address;
} esp_lcd_touch_io_hi8561_config_t;

/**
 * @brief Touch IO configuration structure
 *
 */
#define ESP_LCD_TOUCH_IO_I2C_HI8561_CONFIG()             \
    {                                                   \
        .scl_speed_hz = 100000,                         \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_HI8561_ADDRESS, \
        .control_phase_bytes = 1,                       \
        .dc_bit_offset = 0,                             \
        .lcd_cmd_bits = 16,                             \
        .flags =                                        \
        {                                               \
            .disable_control_phase = 1,                 \
        }                                               \
    }

#ifdef __cplusplus
}
#endif
