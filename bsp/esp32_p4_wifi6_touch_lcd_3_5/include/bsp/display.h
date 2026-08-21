#pragma once
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "sdkconfig.h"

/* LCD color formats */
#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)

/* LCD display color format */
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB565)
/* LCD display color bytes endianess */
#define BSP_LCD_BIGENDIAN           (1)
/* LCD display color bits */
#define BSP_LCD_BITS_PER_PIXEL      (16)
/* LCD display color space */
#define BSP_LCD_COLOR_SPACE         (LCD_RGB_ELEMENT_ORDER_BGR)

#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (480)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 */
typedef struct {
    int max_transfer_sz;    /*!< Maximum transfer size, in bytes. */
} bsp_display_config_t;

/**
 * @brief BSP display return handles
 */
typedef struct {
    esp_lcd_panel_io_handle_t   io;       /*!< ESP LCD IO handle */
    esp_lcd_panel_handle_t      panel;    /*!< ESP LCD panel handle */
    esp_lcd_panel_handle_t      control;  /*!< ESP LCD panel control handle */
} bsp_lcd_handles_t;

/**
 * @brief Create new display panel
 *
 * If you want to free resources allocated by this function, you can use esp_lcd API, ie.:
 *
 * \code{.c}
 * esp_lcd_panel_del(panel);
 * esp_lcd_panel_io_del(io);
 * spi_bus_free(spi_num_from_configuration);
 * \endcode
 *
 * @param[in]  config    display configuration
 * @param[out] ret_panel esp_lcd panel handle
 * @param[out] ret_io    esp_lcd IO handle
 * @return
 *      - ESP_OK         On success
 *      - Else           esp_lcd failure
 */
esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io);

/**
 * @brief Create new display panel
 *
 * If you want to free resources allocated by this function, you can use esp_lcd API, ie.:
 *
 * \code{.c}
 * esp_lcd_panel_del(panel);
 * esp_lcd_panel_io_del(io);
 * spi_bus_free(spi_num_from_configuration);
 * \endcode
 *
 * @param[in]  config      display configuration
 * @param[out] ret_handles all esp_lcd handles in one structure
 * @return
 *      - ESP_OK         On success
 *      - Else           esp_lcd failure
 */
esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles);

/**
 * @brief Initialize display's brightness
 */
esp_err_t bsp_display_brightness_init(void);

/**
 * @brief Set display's brightness
 *
 * @param[in] brightness_percent Brightness in [%]
 */
esp_err_t bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 */
esp_err_t bsp_display_backlight_off(void);

#ifdef __cplusplus
}
#endif
