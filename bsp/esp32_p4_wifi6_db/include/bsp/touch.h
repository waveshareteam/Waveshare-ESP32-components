/**
 * @file
 * @brief BSP touchscreen
 *
 * The GT911 shares the board I2C bus on GPIO7/GPIO8. Its reset and interrupt
 * signals are not connected, so the driver operates in polling mode.
 */

#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g04_display
 *  @{
 */

/**
 * @brief BSP touch configuration
 */
typedef struct {
    struct {
        unsigned int swap_xy: 1;  /*!< Swap X and Y coordinates */
        unsigned int mirror_x: 1; /*!< Mirror X coordinates */
        unsigned int mirror_y: 1; /*!< Mirror Y coordinates */
    } flags;
} bsp_touch_config_t;

/**
 * @brief Create the GT911 touch controller
 *
 * Both valid GT911 I2C addresses, 0x5D and 0x14, are tried.
 *
 * @param[in] config Touch coordinate transformation, or NULL for no transform
 * @param[out] ret_touch Touch handle
 * @return ESP_OK on success, otherwise an esp_lcd or GT911 error
 */
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);

/**
 * @brief Delete the GT911 touch controller and its I2C panel IO
 */
void bsp_touch_delete(void);

/** @} */ // end of display

#ifdef __cplusplus
}
#endif
