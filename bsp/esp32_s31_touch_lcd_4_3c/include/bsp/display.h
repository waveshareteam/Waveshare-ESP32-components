/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_lcd_types.h"

/** \addtogroup g04_display
 *  @{
 */

#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)

#define BSP_LCD_COLOR_FORMAT           (ESP_LCD_COLOR_FORMAT_RGB565)
#define BSP_LCD_BIGENDIAN              (0)
#define BSP_LCD_BITS_PER_PIXEL         (16)
#define BSP_LCD_COLOR_SPACE            (LCD_RGB_ELEMENT_ORDER_RGB)
#define BSP_LCD_H_RES                  (800)
#define BSP_LCD_V_RES                  (480)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int dummy;
} bsp_display_config_t;

typedef struct {
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_handle_t control;
} bsp_lcd_handles_t;

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io);
esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles);
esp_lcd_panel_handle_t bsp_display_get_panel_handle(void);
void bsp_display_delete(void);

esp_err_t bsp_display_brightness_init(void);
esp_err_t bsp_display_brightness_deinit(void);
esp_err_t bsp_display_brightness_set(int brightness_percent);
int bsp_display_brightness_get(void);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);
esp_err_t bsp_set_display_pclk(uint32_t freq_hz);

#ifdef __cplusplus
}
#endif

/** @} */ // end of display
