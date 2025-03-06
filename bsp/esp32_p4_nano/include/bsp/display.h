#pragma once
#include "esp_lcd_types.h"
#include "esp_lcd_mipi_dsi.h"
#include "sdkconfig.h"

/* LCD color formats */
#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)

/* LCD display color format */
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB888)
#else
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB565)
#endif
/* LCD display color bytes endianess */
#define BSP_LCD_BIGENDIAN           (0)
/* LCD display color bits */
#define BSP_LCD_BITS_PER_PIXEL      (16)
/* LCD display color space */
#define BSP_LCD_COLOR_SPACE         (ESP_LCD_COLOR_SPACE_RGB)

#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH
#define BSP_LCD_H_RES              (800)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A
#define BSP_LCD_H_RES              (800)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A
#define BSP_LCD_H_RES              (800)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A
#define BSP_LCD_H_RES              (720)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_480_640_2_8_INCH
#define BSP_LCD_H_RES              (480)
#define BSP_LCD_V_RES              (640)
#elif CONFIG_BSP_LCD_TYPE_800_800_3_4_INCH_C
#define BSP_LCD_H_RES              (800)
#define BSP_LCD_V_RES              (800)
#elif CONFIG_BSP_LCD_TYPE_720_720_4_INCH_C
#define BSP_LCD_H_RES              (720)
#define BSP_LCD_V_RES              (720)
#elif CONFIG_BSP_LCD_TYPE_480_800_4_INCH
#define BSP_LCD_H_RES              (480)
#define BSP_LCD_V_RES              (800)
#elif CONFIG_BSP_LCD_TYPE_720_1280_5_INCH_D
#define BSP_LCD_H_RES              (720)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_720_1560_6_25_INCH
#define BSP_LCD_H_RES              (720)
#define BSP_LCD_V_RES              (1560)
#elif CONFIG_BSP_LCD_TYPE_1024_600_5_INCH_C
#define BSP_LCD_H_RES              (1024)
#define BSP_LCD_V_RES              (600)
#elif CONFIG_BSP_LCD_TYPE_1024_600_7_INCH_C
#define BSP_LCD_H_RES              (1024)
#define BSP_LCD_V_RES              (600)
#elif CONFIG_BSP_LCD_TYPE_400_1280_7_9_INCH
#define BSP_LCD_H_RES              (400)
#define BSP_LCD_V_RES              (1280)
#elif CONFIG_BSP_LCD_TYPE_1280_800_7_INCH_E
#define BSP_LCD_H_RES              (1280)
#define BSP_LCD_V_RES              (800)
#elif CONFIG_BSP_LCD_TYPE_1280_800_8_INCH_C
#define BSP_LCD_H_RES              (1280)
#define BSP_LCD_V_RES              (800)
#elif CONFIG_BSP_LCD_TYPE_1280_800_10_1_INCH_C
#define BSP_LCD_H_RES              (1280)
#define BSP_LCD_V_RES              (800)
#elif CONFIG_BSP_LCD_TYPE_480_1920_8_8_INCH
#define BSP_LCD_H_RES              (480)
#define BSP_LCD_V_RES              (1920)
#elif CONFIG_BSP_LCD_TYPE_320_1480_11_9_INCH
#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (1480)
#endif

#define BSP_LCD_MIPI_DSI_LANE_NUM          (2)    // 2 data lanes
#define BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS (CONFIG_BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS) // 1Gbps

#define BSP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    int dummy;
} bsp_display_config_t;

/**
 * @brief BSP display return handles
 *
 */
typedef struct {
    esp_lcd_dsi_bus_handle_t    mipi_dsi_bus;  /*!< MIPI DSI bus handle */
    esp_lcd_panel_io_handle_t   io;            /*!< ESP LCD IO handle */
    esp_lcd_panel_handle_t      panel;         /*!< ESP LCD panel (color) handle */
    esp_lcd_panel_handle_t      control;       /*!< ESP LCD panel (control) handle */
} bsp_lcd_handles_t;

/**
 * @brief Create new display panel
 *
 * For maximum flexibility, this function performs only reset and initialization of the display.
 * You must turn on the display explicitly by calling esp_lcd_panel_disp_on_off().
 * The display's backlight is not turned on either. You can use bsp_display_backlight_on/off(),
 * bsp_display_brightness_set() (on supported boards) or implement your own backlight control.
 *
 * If you want to free resources allocated by this function, you can use esp_lcd API, ie.:
 *
 * \code{.c}
 * esp_lcd_panel_del(panel);
 * esp_lcd_panel_io_del(io);
 * esp_lcd_del_dsi_bus(mipi_dsi_bus);
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
 * For maximum flexibility, this function performs only reset and initialization of the display.
 * You must turn on the display explicitly by calling esp_lcd_panel_disp_on_off().
 * The display's backlight is not turned on either. You can use bsp_display_backlight_on/off(),
 * bsp_display_brightness_set() (on supported boards) or implement your own backlight control.
 *
 * If you want to free resources allocated by this function, you can use esp_lcd API, ie.:
 *
 * \code{.c}
 * esp_lcd_panel_del(panel);
 * esp_lcd_panel_del(control);
 * esp_lcd_panel_io_del(io);
 * esp_lcd_del_dsi_bus(mipi_dsi_bus);
 * \endcode
 *
 * @param[in]  config    display configuration
 * @param[out] ret_handles all esp_lcd handles in one structure
 * @return
 *      - ESP_OK         On success
 *      - Else           esp_lcd failure
 */
esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles);

/**
 * @brief Initialize display's brightness
 *
 * Brightness is controlled with PWM signal to a pin controlling backlight.
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_brightness_init(void);

/**
 * @brief Set display's brightness
 *
 * Brightness is controlled with PWM signal to a pin controlling backlight.
 * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
 *
 * @param[in] brightness_percent Brightness in [%]
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 *
 * Brightness is controlled with PWM signal to a pin controlling backlight.
 * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 *
 * Brightness is controlled with PWM signal to a pin controlling backlight.
 * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_off(void);

#ifdef __cplusplus
}
#endif
