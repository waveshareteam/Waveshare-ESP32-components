/*
 * SPDX-FileCopyrightText: 2026-2027 Waveshare Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief BSP Touchscreen
 *
 * This file offers API for basic touchscreen initialization.
 * It is useful for users who want to use the touchscreen without the default Graphical Library LVGL.
 *
 * For standard LCD initialization with LVGL graphical library, you can call all-in-one function bsp_display_start().
 */

#pragma once

#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g04_display
 *  @{
 */

/**
 * @brief BSP touch configuration structure
 *
 */
typedef struct {
    void *dummy;    /*!< Prepared for future use. */
} bsp_touch_config_t;


/** @} */ // end of display
#ifdef __cplusplus
}
#endif
