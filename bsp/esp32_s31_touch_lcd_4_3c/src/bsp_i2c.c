/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "bsp_i2c";
static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;

static void bsp_i2c_log_line_levels(const char *stage)
{
    ESP_LOGI(TAG, "%s: SDA(GPIO%d)=%d SCL(GPIO%d)=%d",
             stage,
             BSP_I2C_SDA, gpio_get_level(BSP_I2C_SDA),
             BSP_I2C_SCL, gpio_get_level(BSP_I2C_SCL));
}

static esp_err_t bsp_i2c_recover_bus(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(BSP_I2C_SDA) | BIT64(BSP_I2C_SCL),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&io_conf));

    gpio_set_level(BSP_I2C_SDA, 1);
    gpio_set_level(BSP_I2C_SCL, 1);
    esp_rom_delay_us(10);
    bsp_i2c_log_line_levels("Before recovery");

    if (gpio_get_level(BSP_I2C_SDA) == 1 && gpio_get_level(BSP_I2C_SCL) == 1) {
        BSP_ERROR_CHECK_RETURN_ERR(gpio_reset_pin(BSP_I2C_SDA));
        BSP_ERROR_CHECK_RETURN_ERR(gpio_reset_pin(BSP_I2C_SCL));
        return ESP_OK;
    }

    ESP_LOGW(TAG, "I2C bus is not idle, trying bus recovery pulses");
    for (int i = 0; i < 9; i++) {
        gpio_set_level(BSP_I2C_SCL, 0);
        esp_rom_delay_us(10);
        gpio_set_level(BSP_I2C_SCL, 1);
        esp_rom_delay_us(10);
        if (gpio_get_level(BSP_I2C_SDA) == 1) {
            break;
        }
    }

    gpio_set_level(BSP_I2C_SDA, 0);
    esp_rom_delay_us(10);
    gpio_set_level(BSP_I2C_SCL, 1);
    esp_rom_delay_us(10);
    gpio_set_level(BSP_I2C_SDA, 1);
    esp_rom_delay_us(10);

    bsp_i2c_log_line_levels("After recovery");
    if (gpio_get_level(BSP_I2C_SDA) != 1 || gpio_get_level(BSP_I2C_SCL) != 1) {
        ESP_LOGW(TAG, "I2C bus is still not idle after recovery");
    }
    BSP_ERROR_CHECK_RETURN_ERR(gpio_reset_pin(BSP_I2C_SDA));
    BSP_ERROR_CHECK_RETURN_ERR(gpio_reset_pin(BSP_I2C_SCL));
    return ESP_OK;
}

esp_err_t bsp_i2c_init(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_recover_bus());

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_config, &i2c_handle));

    i2c_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_handle = NULL;
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    bsp_i2c_init();
    return i2c_handle;
}
