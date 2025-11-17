/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include "sdkconfig.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_oled_ssd1315.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_compiler.h"

static const char *TAG = "lcd_panel.ssd1315";

// SSD1315 commands
#define SSD1315_CMD_SET_MEMORY_ADDR_MODE  0x20
#define SSD1315_CMD_SET_COLUMN_RANGE      0x21
#define SSD1315_CMD_SET_PAGE_RANGE        0x22
#define SSD1315_CMD_SET_STARTLINE         0x40
#define SSD1315_CMD_SET_CONTRAST          0x81
#define SSD1315_CMD_SET_CHARGE_PUMP       0x8D
#define SSD1315_CMD_MIRROR_X_OFF          0xA0
#define SSD1315_CMD_MIRROR_X_ON           0xA1
#define SSD1315_CMD_DISPLAYALLON_RESUME   0xA4
#define SSD1315_CMD_INVERT_OFF            0xA6
#define SSD1315_CMD_INVERT_ON             0xA7
#define SSD1315_CMD_SET_MULTIPLEX         0xA8
#define SSD1315_CMD_DISP_OFF              0xAE
#define SSD1315_CMD_DISP_ON               0xAF
#define SSD1315_CMD_MIRROR_Y_OFF          0xC0
#define SSD1315_CMD_MIRROR_Y_ON           0xC8
#define SSD1315_CMD_SET_DISPLAYOFFSET     0xD3
#define SSD1315_CMD_SET_DISPLAYCLOCKDIV   0xD5
#define SSD1315_CMD_SET_PRECHARGE         0xD9
#define SSD1315_CMD_SET_COMPINS           0xDA
#define SSD1315_CMD_SET_VCOMDETECT        0xDB


static esp_err_t panel_ssd1315_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1315_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1315_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1315_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1315_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1315_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1315_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1315_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1315_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    uint8_t height;
    gpio_num_t reset_gpio_num;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    bool reset_level;
    bool swap_axes;
} ssd1315_panel_t;

esp_err_t esp_lcd_new_panel_ssd1315(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    esp_err_t ret = ESP_OK;
    ssd1315_panel_t *ssd1315 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(panel_dev_config->bits_per_pixel == 1, ESP_ERR_INVALID_ARG, err, TAG, "bpp must be 1");
    esp_lcd_panel_ssd1315_config_t *ssd1315_spec_config = (esp_lcd_panel_ssd1315_config_t *)panel_dev_config->vendor_config;
    // leak detection of ssd1315 because saving ssd1315->base address
    ESP_COMPILER_DIAGNOSTIC_PUSH_IGNORE("-Wanalyzer-malloc-leak")
    ssd1315 = calloc(1, sizeof(ssd1315_panel_t));
    ESP_GOTO_ON_FALSE(ssd1315, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1315 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    ssd1315->io = io;
    ssd1315->bits_per_pixel = panel_dev_config->bits_per_pixel;
    ssd1315->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1315->reset_level = panel_dev_config->flags.reset_active_high;
    ssd1315->height = ssd1315_spec_config ? ssd1315_spec_config->height : 64;
    ssd1315->base.del = panel_ssd1315_del;
    ssd1315->base.reset = panel_ssd1315_reset;
    ssd1315->base.init = panel_ssd1315_init;
    ssd1315->base.draw_bitmap = panel_ssd1315_draw_bitmap;
    ssd1315->base.invert_color = panel_ssd1315_invert_color;
    ssd1315->base.set_gap = panel_ssd1315_set_gap;
    ssd1315->base.mirror = panel_ssd1315_mirror;
    ssd1315->base.swap_xy = panel_ssd1315_swap_xy;
    ssd1315->base.disp_on_off = panel_ssd1315_disp_on_off;
    *ret_panel = &(ssd1315->base);
    ESP_LOGD(TAG, "new ssd1315 panel @%p", ssd1315);

    return ESP_OK;

err:
    if (ssd1315) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ssd1315);
    }
    return ret;
    ESP_COMPILER_DIAGNOSTIC_POP("-Wanalyzer-malloc-leak")
}

static esp_err_t panel_ssd1315_del(esp_lcd_panel_t *panel)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    if (ssd1315->reset_gpio_num >= 0) {
        gpio_reset_pin(ssd1315->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ssd1315 panel @%p", ssd1315);
    free(ssd1315);
    return ESP_OK;
}

static esp_err_t panel_ssd1315_reset(esp_lcd_panel_t *panel)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);

    // perform hardware reset
    if (ssd1315->reset_gpio_num >= 0) {
        gpio_set_level(ssd1315->reset_gpio_num, ssd1315->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ssd1315->reset_gpio_num, !ssd1315->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t panel_ssd1315_init(esp_lcd_panel_t *panel)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1315->io;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_MULTIPLEX, (uint8_t[]) {
        ssd1315->height - 1 // set multiplex ratio
    }, 1), TAG, "io tx param SSD1315_CMD_SET_MULTIPLEX failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_COMPINS, (uint8_t[1]) {
        ssd1315->height == 64 ? 0x12 : 0x02 // set COM pins hardware configuration
    }, 1), TAG, "io tx param SSD1315_CMD_SET_COMPINS failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_DISP_OFF, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_DISP_OFF failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_DISPLAYCLOCKDIV, (uint8_t[]) {
        0x80 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_SETDISPLAYCLOCKDIV failed");  
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_DISPLAYOFFSET, (uint8_t[]) {
        0x00 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_DISPLAYOFFSET failed");    
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_STARTLINE, NULL , 0), TAG, 
                        "io tx param SSD1315_CMD_SET_STARTLINE failed");          
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_MEMORY_ADDR_MODE, (uint8_t[]) {
        0x00 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_MEMORY_ADDR_MODE failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_CHARGE_PUMP, (uint8_t[]) {
        0x14 // enable charge pump
    }, 1), TAG, "io tx param SSD1315_CMD_SET_CHARGE_PUMP failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_MIRROR_X_OFF | 0x01, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_MIRROR_X_OFF failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_MIRROR_Y_OFF | 0x08, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_MIRROR_Y_OFF failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_COMPINS, (uint8_t[]) {
        0x12 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_COMPINS failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_CONTRAST, (uint8_t[]) {
        0xCF // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_CONTRAST failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_PRECHARGE, (uint8_t[]) {
        0xF1 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_PRECHARGE failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_VCOMDETECT, (uint8_t[]) {
        0x40 // horizontal addressing mode
    }, 1), TAG, "io tx param SSD1315_CMD_SET_VCOMDETECT failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_DISPLAYALLON_RESUME, NULL , 0), TAG, 
                        "io tx param SSD1315_CMD_DISPLAYALLON_RESUME failed"); 
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_INVERT_OFF, NULL , 0), TAG, 
                        "io tx param SSD1315_CMD_INVERT_OFF failed");                     
    return ESP_OK;
}

static esp_err_t panel_ssd1315_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1315->io;

    // adding extra gap
    x_start += ssd1315->x_gap;
    x_end += ssd1315->x_gap;
    y_start += ssd1315->y_gap;
    y_end += ssd1315->y_gap;

    if (ssd1315->swap_axes) {
        int x = x_start;
        x_start = y_start;
        y_start = x;
        x = x_end;
        x_end = y_end;
        y_end = x;
    }

    // one page contains 8 rows (COMs)
    uint8_t page_start = y_start / 8;
    uint8_t page_end = (y_end - 1) / 8;
    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_COLUMN_RANGE, (uint8_t[]) {
        (x_start & 0x7F),
        ((x_end - 1) & 0x7F),
    }, 2), TAG, "io tx param SSD1315_CMD_SET_COLUMN_RANGE failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1315_CMD_SET_PAGE_RANGE, (uint8_t[]) {
        (page_start & 0x07),
        (page_end & 0x07),
    }, 2), TAG, "io tx param SSD1315_CMD_SET_PAGE_RANGE failed");
    // transfer frame buffer
    size_t len = (y_end - y_start) * (x_end - x_start) * ssd1315->bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, -1, color_data, len), TAG, "io tx color failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1315_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1315->io;
    int command = 0;
    if (invert_color_data) {
        command = SSD1315_CMD_INVERT_ON;
    } else {
        command = SSD1315_CMD_INVERT_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_INVERT_ON/OFF failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1315_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1315->io;

    int command = 0;
    if (mirror_x) {
        command = SSD1315_CMD_MIRROR_X_ON;
    } else {
        command = SSD1315_CMD_MIRROR_X_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_MIRROR_X_ON/OFF failed");
    if (mirror_y) {
        command = SSD1315_CMD_MIRROR_Y_ON;
    } else {
        command = SSD1315_CMD_MIRROR_Y_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_MIRROR_Y_ON/OFF failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1315_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    ssd1315->swap_axes = swap_axes;

    return ESP_OK;
}

static esp_err_t panel_ssd1315_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    ssd1315->x_gap = x_gap;
    ssd1315->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ssd1315_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1315_panel_t *ssd1315 = __containerof(panel, ssd1315_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1315->io;
    int command = 0;
    if (on_off) {
        command = SSD1315_CMD_DISP_ON;
    } else {
        command = SSD1315_CMD_DISP_OFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param SSD1315_CMD_DISP_ON/OFF failed");
    // SEG/COM will be ON/OFF after 100ms after sending DISP_ON/OFF command
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}