/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_lcd_touch_hi8561.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "HI8561";

/* Compile-time array bounds. */
#define HI8561_FW_PACKAGE_SIZE 60
#define HI8561_MAX_FINGERS     10

static const size_t s_fw_offset_checksum = 58;

typedef struct {
    esp_lcd_touch_t touch;
    i2c_master_dev_handle_t i2c_dev;
    uint32_t report_address;
} hi8561_touch_t;

static esp_err_t hi8561_read_register(const hi8561_touch_t *driver, uint32_t address, uint8_t *buffer, size_t size)
{
    const uint8_t command[] = {
        0xF3, (uint8_t)(address >> 24), (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address, 0x03,
    };
    return i2c_master_transmit_receive(driver->i2c_dev, command, sizeof(command), buffer, size, 50);
}

static esp_err_t hi8561_hardware_reset(esp_lcd_touch_handle_t tp)
{
    if (tp->config.rst_gpio_num == GPIO_NUM_NC) {
        return ESP_OK;
    }

    const gpio_config_t rst_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(tp->config.rst_gpio_num),
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), TAG, "RST GPIO config failed");

    /* Follow TP-reset sequence: high (5ms) -> low (20ms) -> high (150ms). */
    const uint8_t active_level = tp->config.levels.reset;
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !active_level), TAG, "RST set level failed");
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, active_level), TAG, "RST set level failed");
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !active_level), TAG, "RST set level failed");
    vTaskDelay(pdMS_TO_TICKS(150));

    return ESP_OK;
}

static esp_err_t hi8561_interrupt_init(esp_lcd_touch_handle_t tp)
{
    if (tp->config.int_gpio_num == GPIO_NUM_NC) {
        return ESP_OK;
    }

    const gpio_config_t int_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = (tp->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
        .pin_bit_mask = BIT64(tp->config.int_gpio_num),
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "INT GPIO config failed");

    if (tp->config.interrupt_callback != NULL) {
        return esp_lcd_touch_register_interrupt_callback(tp, tp->config.interrupt_callback);
    }
    return ESP_OK;
}

static bool hi8561_verify_checksum(const uint8_t *buf)
{
    uint8_t sum = 0;
    for (size_t i = 0; i <= s_fw_offset_checksum; i++) {
        sum += buf[i];
    }
    return (sum == 0);
}

static void hi8561_clear_points(esp_lcd_touch_handle_t tp)
{
    portENTER_CRITICAL(&tp->data.lock);
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);
}

static void hi8561_parse_points(esp_lcd_touch_handle_t tp, const uint8_t *buf, uint8_t finger_num, uint8_t max_points)
{
    portENTER_CRITICAL(&tp->data.lock);
    uint8_t valid_points = 0;
    for (uint8_t slot = 0; slot < HI8561_MAX_FINGERS && valid_points < finger_num && valid_points < max_points; slot++) {
        const size_t base = 3 + slot * 5;
        uint16_t x_raw = ((uint16_t)buf[base] << 8) | buf[base + 1];
        uint16_t y_raw = ((uint16_t)buf[base + 2] << 8) | buf[base + 3];
        uint8_t p_raw = buf[base + 4];

        if (x_raw >= tp->config.x_max || y_raw >= tp->config.y_max || p_raw == 0) {
            continue;
        }

        tp->data.coords[valid_points].x = x_raw;
        tp->data.coords[valid_points].y = y_raw;
        tp->data.coords[valid_points].strength = p_raw;
        tp->data.coords[valid_points].track_id = slot;
        valid_points++;
    }
    tp->data.points = valid_points;
    portEXIT_CRITICAL(&tp->data.lock);
}

static esp_err_t esp_lcd_touch_hi8561_read_data(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    hi8561_touch_t *driver = (hi8561_touch_t *)tp;
    const uint8_t max_points = CONFIG_ESP_LCD_TOUCH_MAX_POINTS < HI8561_MAX_FINGERS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : HI8561_MAX_FINGERS;

    uint8_t buf[HI8561_FW_PACKAGE_SIZE];
    esp_err_t err = hi8561_read_register(driver, driver->report_address, buf, sizeof(buf));
    if (err != ESP_OK) {
        hi8561_clear_points(tp);
        ESP_LOGE(TAG, "I2C read error! code: %#X", err);
        return err;
    }

    const uint8_t finger_num = buf[0];
    if (finger_num == 0 || finger_num > HI8561_MAX_FINGERS) {
        if (finger_num > HI8561_MAX_FINGERS) {
            ESP_LOGW(TAG, "Invalid finger count %u, hdr=[%02x %02x %02x]", finger_num, buf[0], buf[1], buf[2]);
        }
        hi8561_clear_points(tp);
        return ESP_OK;
    }

    if (!hi8561_verify_checksum(buf)) {
        ESP_LOGW(TAG, "Invalid checksum: cnt=%u csum=0x%02x", buf[1], buf[s_fw_offset_checksum]);
        return ESP_OK;
    }

    hi8561_parse_points(tp, buf, finger_num, max_points);
    return ESP_OK;
}

static bool esp_lcd_touch_hi8561_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    ESP_RETURN_ON_FALSE(tp != NULL, false, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(x != NULL, false, TAG, "Pointer to the x coordinates array can't be NULL");
    ESP_RETURN_ON_FALSE(y != NULL, false, TAG, "Pointer to the y coordinates array can't be NULL");
    ESP_RETURN_ON_FALSE(point_num != NULL, false, TAG, "Pointer to number of touch points can't be NULL");
    ESP_RETURN_ON_FALSE(max_point_num > 0, false, TAG, "Array size must be equal or larger than 1");

    portENTER_CRITICAL(&tp->data.lock);
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;
        if (strength != NULL) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_hi8561_del(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        if (tp->config.interrupt_callback != NULL) {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    hi8561_touch_t *driver = (hi8561_touch_t *)tp;
    if (driver->i2c_dev != NULL) {
        i2c_master_bus_rm_device(driver->i2c_dev);
    }

    free(driver);
    return ESP_OK;
}

esp_err_t esp_lcd_touch_new_i2c_hi8561(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the touch controller configuration can't be NULL");
    ESP_RETURN_ON_FALSE(out_touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the touch controller handle can't be NULL");

    const esp_lcd_touch_io_hi8561_config_t *hi8561_cfg = (const esp_lcd_touch_io_hi8561_config_t *)config->driver_data;
    ESP_RETURN_ON_FALSE(hi8561_cfg != NULL && hi8561_cfg->i2c_bus != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing touch I2C bus handle");

    hi8561_touch_t *driver = heap_caps_calloc(1, sizeof(*driver), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_NO_MEM, TAG, "No mem for HI8561 controller");

    esp_lcd_touch_handle_t tp = &driver->touch;
    tp->io = io;
    tp->read_data = esp_lcd_touch_hi8561_read_data;
    tp->get_xy = esp_lcd_touch_hi8561_get_xy;
    tp->del = esp_lcd_touch_hi8561_del;
    tp->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&tp->config, config, sizeof(esp_lcd_touch_config_t));

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = (hi8561_cfg->dev_addr != 0) ? hi8561_cfg->dev_addr : ESP_LCD_TOUCH_IO_I2C_HI8561_ADDRESS,
        .scl_speed_hz = (hi8561_cfg->scl_speed_hz != 0) ? hi8561_cfg->scl_speed_hz : 200000,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(hi8561_cfg->i2c_bus, &dev_cfg, &driver->i2c_dev), err, TAG, "Failed to add HI8561 I2C device");

    ESP_GOTO_ON_ERROR(hi8561_hardware_reset(tp), err, TAG, "HI8561 reset failed");
    ESP_GOTO_ON_ERROR(hi8561_interrupt_init(tp), err, TAG, "HI8561 interrupt init failed");

    driver->report_address = 0x20011120;
    ESP_LOGI(TAG, "Report address: 0x%08" PRIx32, driver->report_address);

    *out_touch = tp;
    return ESP_OK;

err:
    ESP_LOGE(TAG, "Error (0x%x)! Touch controller HI8561 initialization failed!", ret);
    esp_lcd_touch_hi8561_del(tp);
    return ret;
}
