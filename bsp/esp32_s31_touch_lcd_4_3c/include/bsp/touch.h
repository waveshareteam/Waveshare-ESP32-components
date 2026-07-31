/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "esp_lcd_touch.h"

#define BSP_TOUCH_I2C_ADDRESS       (0x5d)
#define BSP_TOUCH_I2C_ADDRESS_ALT   (0x14)

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g04_display
 *  @{
 */

typedef struct {
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
} bsp_touch_config_t;

/** Reset GT911, select the primary I2C address, and release INT as an input. */
esp_err_t bsp_touch_reset(void);
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);

/** @} */ // end of display

#ifdef __cplusplus
}
#endif
