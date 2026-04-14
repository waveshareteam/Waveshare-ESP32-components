#pragma once
#include "iot_button.h"
#include "button_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP touch configuration structure
 *
 */
typedef struct {
    button_handle_t prev;
    button_handle_t next;
    button_handle_t enter;
} esp_buttons_handle_t;

/**
 * @brief Create new touchscreen
 *
 * If you want to free resources allocated by this function, you can use esp_lcd_touch API, ie.:
 *
 * \code{.c}
 * esp_lcd_touch_del(tp);
 * \endcode
 *
 * @param[in]  config    touch configuration
 * @param[out] ret_touch esp_lcd_touch touchscreen handle
 * @return
 *      - ESP_OK         On success
 *      - Else           esp_lcd_touch failure
 */
esp_err_t bsp_buttons_new(const bsp_display_cfg_t *cfg, esp_buttons_handle_t *ret_buttons);

#ifdef __cplusplus
}
#endif