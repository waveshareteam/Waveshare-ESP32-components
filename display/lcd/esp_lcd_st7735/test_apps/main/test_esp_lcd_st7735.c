/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_check.h"
#include "esp_log.h"
#include "string.h"
#include "esp_lcd_st7735.h"


/* LCD Dimensions */
#define LCD_H_RES          (132)    // LCD horizontal resolution
#define LCD_V_RES          (132)    // LCD vertical resolution

/* LCD SPI Configuration */
#define LCD_SPI_NUM        (SPI3_HOST)       // SPI host to use
#define LCD_PIXEL_CLK_HZ   (40 * 1000 * 1000) // SPI clock frequency (40MHz)
#define LCD_CMD_BITS       (8)                // Command bit width
#define LCD_PARAM_BITS     (8)                // Parameter bit width
#define LCD_COLOR_SPACE    (ESP_LCD_COLOR_SPACE_RGB) // Color space
#define LCD_BITS_PER_PIXEL (16)               // Pixel bit width (RGB565 format)
#define LCD_BL_ON_LEVEL    (1)                // Backlight on level (1=high level, 0=low level)

/* LCD Pin Definitions */
#define LCD_GPIO_SCLK      (GPIO_NUM_6) // SPI clock pin
#define LCD_GPIO_MOSI      (GPIO_NUM_7) // SPI data output pin (ST7735 simplex, MISO not required)
#define LCD_GPIO_RST       (GPIO_NUM_38) // Reset pin
#define LCD_GPIO_DC        (GPIO_NUM_4) // Command/data distinction pin
#define LCD_GPIO_CS        (GPIO_NUM_5) // Chip select pin
#define LCD_GPIO_BL        (GPIO_NUM_40) // Backlight pin

static const char *TAG = "LCD_DRIVER";       
static esp_lcd_panel_io_handle_t lcd_io = NULL;  // LCD panel IO handle (SPI communication layer)
esp_lcd_panel_handle_t lcd_panel = NULL;  // LCD panel handle (ST7735 driver layer)

void lcd_fill_screen(esp_lcd_panel_handle_t panel_handle, uint16_t color)
{
    uint32_t pixel_count = LCD_H_RES * LCD_V_RES;
    uint8_t *color_buf = heap_caps_malloc(pixel_count * 2, MALLOC_CAP_DMA);
    if (color_buf == NULL) {
        ESP_LOGE("LCD", "malloc failed");
        return;
    }

    uint8_t color_high = (color >> 8) & 0xFF; 
    uint8_t color_low = color & 0xFF;         

    for (uint32_t i = 0; i < pixel_count; i++) {
        color_buf[i * 2]     = color_high;
        color_buf[i * 2 + 1] = color_low;
    }
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0,LCD_H_RES, LCD_V_RES, (uint16_t *)color_buf);
    free(color_buf);
}


static esp_err_t lcd_backlight_init(void)
{
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_GPIO_BL
    };
    return gpio_config(&bk_gpio_config);
}


static esp_err_t lcd_spi_bus_init(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_GPIO_SCLK,
        .mosi_io_num = LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,  // ST7735 doesn't need MISO (simplex communication)
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t) 
    };
    return spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO);
}


esp_err_t lcd_init(void)
{
    esp_err_t ret = ESP_OK;
    // 1. Initialize backlight pin
    ESP_GOTO_ON_ERROR(lcd_backlight_init(), err, TAG, "Backlight init failed");

    // 2. Initialize SPI bus
    ESP_LOGD(TAG, "Initialize SPI bus for ST7735");
    ESP_GOTO_ON_ERROR(lcd_spi_bus_init(), err, TAG, "SPI bus init failed");

    // 3. Install LCD panel IO (SPI protocol adaptation)
    ESP_LOGD(TAG, "Install LCD panel IO (SPI)");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_GPIO_DC,
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), 
                      err, TAG, "Panel IO init failed");

    // 4. Install ST7735 LCD driver
    ESP_LOGD(TAG, "Install ST7735 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .color_space = LCD_COLOR_SPACE,
        .bits_per_pixel = LCD_BITS_PER_PIXEL
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7735(lcd_io, &panel_config, &lcd_panel), 
                      err, TAG, "ST7735 driver init failed");

    // 5. Reset and initialize ST7735 panel
    ESP_LOGD(TAG, "Reset and init ST7735 panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(lcd_panel), err, TAG, "Panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(lcd_panel), err, TAG, "Panel init failed");
    
    // ST7735 has default color inversion; enable color correction
    ESP_GOTO_ON_ERROR(esp_lcd_panel_invert_color(lcd_panel, true), err, TAG, "Invert color failed");
    // Turn on LCD display
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(lcd_panel, true), err, TAG, "Turn on display failed");
    lcd_fill_screen(lcd_panel, 0x0000);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 6. Turn on LCD backlight
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGD(TAG, "Turn on LCD backlight");
    ESP_GOTO_ON_ERROR(gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL), err, TAG, "Turn on backlight failed");

    ESP_LOGI(TAG, "ST7735 LCD init success");
    return ret;

err:
    // Initialization failed: Release allocated resources to avoid memory leak
    if (lcd_panel != NULL) {
        esp_lcd_panel_del(lcd_panel);
        lcd_panel = NULL;
    }
    if (lcd_io != NULL) {
        esp_lcd_panel_io_del(lcd_io);
        lcd_io = NULL;
    }
    spi_bus_free(LCD_SPI_NUM);
    ESP_LOGE(TAG, "ST7735 LCD init failed (err: %s)", esp_err_to_name(ret));
    return ret;
}


void app_main(void)
{
    lcd_init();

    while(1) {

        lcd_fill_screen(lcd_panel, 0xF800); // 填充红色
        vTaskDelay(pdMS_TO_TICKS(2000));

        lcd_fill_screen(lcd_panel, 0x07E0); // 填充绿色
        vTaskDelay(pdMS_TO_TICKS(2000));

        lcd_fill_screen(lcd_panel, 0x001F); // 填充蓝色
        vTaskDelay(pdMS_TO_TICKS(2000));

        lcd_fill_screen(lcd_panel, 0xFFE0); // 填充黄色
        vTaskDelay(pdMS_TO_TICKS(2000));

        lcd_fill_screen(lcd_panel, 0xFFFF); // 填充白色
    }
}