/*
 * SPDX-FileCopyrightText: 2026 Waveshare Team
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
} bsp_touch_config_t;

esp_err_t bsp_touch_new(const bsp_touch_config_t *config,
                        esp_lcd_touch_handle_t *ret_touch);

#ifdef __cplusplus
}
#endif
