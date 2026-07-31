/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "esp_check.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"

static const char *TAG = "ESP32-S31-Touch-LCD-4.3C";
static bool sd_control_initialized;

#if defined(CONFIG_BSP_SD_CTRL_ACTIVE_LOW)
#define BSP_SD_DISABLED_LEVEL  1
#define BSP_SD_ENABLED_LEVEL   0
#else
#define BSP_SD_DISABLED_LEVEL  0
#define BSP_SD_ENABLED_LEVEL   1
#endif

static esp_err_t bsp_sd_control_init(void)
{
    if (sd_control_initialized) {
        return ESP_OK;
    }

    gpio_set_level(BSP_SD_EN, BSP_SD_DISABLED_LEVEL);
    const gpio_config_t io_config = {
        .pin_bit_mask = BIT64(BSP_SD_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "SD_CTRL GPIO config failed");

    sd_control_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    switch (feature) {
    case BSP_FEATURE_SD: {
        ESP_RETURN_ON_ERROR(bsp_sd_control_init(), TAG, "SD_CTRL init failed");
        const int level = enable ? BSP_SD_ENABLED_LEVEL : BSP_SD_DISABLED_LEVEL;
        return gpio_set_level(BSP_SD_EN, level);
    }
    default:
        return ESP_ERR_INVALID_ARG;
    }
}
