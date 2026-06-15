/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_hi8561.h"

static const char *TAG = "HI8561";

/* HI8561 registers */
#define MEMORY_ADDRESS_ERAM                 0x20011000
#define MAX_DSRAM_NUM                       25
#define DSRAM_SECTION_INFO_START_ADDRESS    (MEMORY_ADDRESS_ERAM + 4)
#define ESRAM_NUM_START_ADDRESS             (DSRAM_SECTION_INFO_START_ADDRESS + MAX_DSRAM_NUM * 8)
#define ESRAM_SECTION_INFO_START_ADDRESS    (ESRAM_NUM_START_ADDRESS + 4)
#define MEMORY_ERAM_SIZE                    (4 * 1024)

/* HI8561 support key num */
#define ESP_HI8561_TOUCH_MAX_BUTTONS         (4)

#define TOUCH_POINT_ADDRESS_OFFSET          3
#define SINGLE_TOUCH_POINT_DATA_SIZE        5
#define MAX_TOUCH_FINGER_COUNT              10

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_hi8561_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_hi8561_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength,
                                       uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_hi8561_del(esp_lcd_touch_handle_t tp);

/* I2C read/write */
static esp_err_t touch_hi8561_i2c_read(esp_lcd_touch_handle_t tp, uint8_t *sbuff, uint8_t sbuff_len, uint8_t *data, uint8_t len);
static esp_err_t touch_hi8561_i2c_write(esp_lcd_touch_handle_t tp, uint8_t *buff, uint8_t buff_len);

/* HI8561 reset */
static esp_err_t touch_hi8561_reset(esp_lcd_touch_handle_t tp);
/* Read status and config register */
static esp_err_t touch_hi8561_read_info_start_address(esp_lcd_touch_handle_t tp);

static uint32_t touch_info_start_address = 0;

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_hi8561(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config,
                                      esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(io != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller io handle can't be NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the touch controller configuration can't be NULL");
    ESP_RETURN_ON_FALSE(out_touch != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the touch controller handle can't be NULL");

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_hi8561 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_hi8561, ESP_ERR_NO_MEM, err, TAG, "no mem for HI8561 controller");

    /* Communication interface */
    esp_lcd_touch_hi8561->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_hi8561->read_data = esp_lcd_touch_hi8561_read_data;
    esp_lcd_touch_hi8561->get_xy = esp_lcd_touch_hi8561_get_xy;
    esp_lcd_touch_hi8561->del = esp_lcd_touch_hi8561_del;

    /* Mutex */
    esp_lcd_touch_hi8561->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_hi8561->config, config, sizeof(esp_lcd_touch_config_t));
    esp_lcd_touch_io_hi8561_config_t *hi8561_config = (esp_lcd_touch_io_hi8561_config_t *)
            esp_lcd_touch_hi8561->config.driver_data;

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_hi8561->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_hi8561->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    if (hi8561_config && esp_lcd_touch_hi8561->config.rst_gpio_num != GPIO_NUM_NC
            && esp_lcd_touch_hi8561->config.int_gpio_num != GPIO_NUM_NC) {
        /* Prepare pin for touch controller int */
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_down_en = 0,
            .pull_up_en = 1,
            .pin_bit_mask = BIT64(esp_lcd_touch_hi8561->config.int_gpio_num),
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_hi8561->config.rst_gpio_num, esp_lcd_touch_hi8561->config.levels.reset),
                            TAG, "GPIO set level error!");
        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_hi8561->config.int_gpio_num, 0), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));

        ESP_RETURN_ON_ERROR(gpio_set_level(esp_lcd_touch_hi8561->config.rst_gpio_num, !esp_lcd_touch_hi8561->config.levels.reset),
                            TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));

        vTaskDelay(pdMS_TO_TICKS(50));
    } else {
        ESP_LOGI(TAG, "I2C address initialization procedure skipped - using default GT9xx setup");
        /* Reset controller */
        ret = touch_hi8561_reset(esp_lcd_touch_hi8561);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "HI8561 reset failed");
    }

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_hi8561->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (esp_lcd_touch_hi8561->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(esp_lcd_touch_hi8561->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        /* Register interrupt callback */
        if (esp_lcd_touch_hi8561->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_hi8561, esp_lcd_touch_hi8561->config.interrupt_callback);
        }
    }

    /* Read status and config info */
    ret = touch_hi8561_read_info_start_address(esp_lcd_touch_hi8561);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "HI8561 init failed");

    *out_touch = esp_lcd_touch_hi8561;

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller HI8561 initialization failed!", ret);
        if (esp_lcd_touch_hi8561) {
            esp_lcd_touch_hi8561_del(esp_lcd_touch_hi8561);
        }
    }

    return ret;
}

static uint8_t get_finger_count(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t finger_nums = 0;

    uint8_t buffer[6] = {
        0xF3,
        (uint8_t)(touch_info_start_address >> 24),
        (uint8_t)(touch_info_start_address >> 16),
        (uint8_t)(touch_info_start_address >> 8),
        (uint8_t)(touch_info_start_address),
        0x03,
    };

    err = touch_hi8561_i2c_read(tp, buffer, sizeof(buffer), &finger_nums, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");
    return finger_nums;
}

static esp_err_t esp_lcd_touch_hi8561_read_data(esp_lcd_touch_handle_t tp)
{
	ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    esp_err_t err;
    uint8_t finger_num = 0;

    // 1. Get the number of currently active touch points
    finger_num = get_finger_count(tp);

    // 2. If no fingers are detected, notify the framework and return normally
    if (finger_num == 0) {
        portENTER_CRITICAL(&tp->data.lock);
        tp->data.points = 0; // CRITICAL: Clear point count on release to avoid sticky coordinates
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_OK; // Return ESP_OK to handle the release action correctly
    }

    // Limit the maximum number of fingers to prevent memory overflow
    if (finger_num > MAX_TOUCH_FINGER_COUNT) {
        finger_num = MAX_TOUCH_FINGER_COUNT;
    }

    // 3. Construct the I2C read command buffer (sending internal memory address)
    uint8_t buffer[6] = {
        0xF3,
        (uint8_t)(touch_info_start_address >> 24),
        (uint8_t)(touch_info_start_address >> 16),
        (uint8_t)(touch_info_start_address >> 8),
        (uint8_t)(touch_info_start_address),
        0x03,
    };

    // Calculate the total number of bytes to read for all active fingers
    const uint8_t touch_point_pointer = TOUCH_POINT_ADDRESS_OFFSET + (finger_num * SINGLE_TOUCH_POINT_DATA_SIZE);

    // Allocate a temporary buffer using variable-length array (VLA)
    uint8_t buf[touch_point_pointer];
    memset(buf, 0, touch_point_pointer);

    // 4. Perform I2C read operation to fetch data for all points at once
    err = touch_hi8561_i2c_read(tp, buffer, sizeof(buffer), buf, touch_point_pointer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error! code: %#X", err);
        return err;
    }

    // 5. Fill coordinates into the core esp_lcd_touch structure
    portENTER_CRITICAL(&tp->data.lock);

    // Notify the upper layer framework about the total number of valid touch points
    tp->data.points = finger_num;

    for (int i = 0; i < finger_num; i++) {
        // Calculate the base index for the i-th touch point data block
        int base_idx = TOUCH_POINT_ADDRESS_OFFSET + (i * SINGLE_TOUCH_POINT_DATA_SIZE);

        // Parse coordinates from raw data bytes (High byte left shifted by 8 | Low byte)
        uint16_t x_raw = ((uint16_t)buf[base_idx] << 8) | buf[base_idx + 1];
        uint16_t y_raw = ((uint16_t)buf[base_idx + 2] << 8) | buf[base_idx + 3];
        uint8_t p_raw  = buf[base_idx + 4];

        // Check for edge touch flag inherited from the reference implementation (X=65535, Y=65535, P=0)
        if (x_raw == 65535 && y_raw == 65535 && p_raw == 0) {
            // Edge touch detected, invalidate this specific point coordinate
            tp->data.coords[i].x = 0;
            tp->data.coords[i].y = 0;
            tp->data.coords[i].strength = 0;
        } else {
            // Valid touch data, assign to esp_lcd_touch structure
            tp->data.coords[i].x = x_raw;
            tp->data.coords[i].y = y_raw;
            tp->data.coords[i].strength = p_raw; // Map 'p' (pressure) to 'strength'
        }

        // Keep track_id as loop index or parse if extra byte exists
        tp->data.coords[i].track_id = i; 
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}

static bool esp_lcd_touch_hi8561_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength,
                                       uint8_t *point_num, uint8_t max_point_num)
{
    ESP_RETURN_ON_FALSE(tp != NULL, false, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(x != NULL, false, TAG, "Pointer to the x coordinates array can't be NULL");
    ESP_RETURN_ON_FALSE(y != NULL, false, TAG, "Pointer to the y coordinates array can't be NULL");
    ESP_RETURN_ON_FALSE(point_num != NULL, false, TAG, "Pointer to number of touch points can't be NULL");
    ESP_RETURN_ON_FALSE(max_point_num > 0, false, TAG, "Array size must be equal or larger than 1");

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_hi8561_del(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
        if (tp->config.interrupt_callback) {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

/* Reset controller */
static esp_err_t touch_hi8561_reset(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t touch_hi8561_read_info_start_address(esp_lcd_touch_handle_t tp)
{
    uint8_t address_buff[48] = {0};
    uint8_t buffer[6] = {
        0xF3,
        (uint8_t)(ESRAM_SECTION_INFO_START_ADDRESS >> 24),
        (uint8_t)(ESRAM_SECTION_INFO_START_ADDRESS >> 16),
        (uint8_t)(ESRAM_SECTION_INFO_START_ADDRESS >> 8),
        (uint8_t)(ESRAM_SECTION_INFO_START_ADDRESS),
        0x03,
    };

    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");
    
    touch_hi8561_i2c_read(tp, buffer, sizeof(buffer), address_buff, sizeof(address_buff));

    touch_info_start_address = address_buff[8] + (address_buff[8 + 1] << 8) + (address_buff[8 + 2] << 16) + (address_buff[8 + 3] << 24);
    if ((touch_info_start_address < MEMORY_ADDRESS_ERAM) || 
        (touch_info_start_address >= (MEMORY_ADDRESS_ERAM + MEMORY_ERAM_SIZE)))
    {
        ESP_LOGE(TAG, "read info start address get error\n");
        touch_info_start_address = 0;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t touch_hi8561_i2c_read(esp_lcd_touch_handle_t tp, uint8_t *sbuff, uint8_t sbuff_len, uint8_t *data, uint8_t len)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the data array can't be NULL");

    i2c_master_dev_handle_t iic_dev = (i2c_master_dev_handle_t)tp->config.user_data;
    return i2c_master_transmit_receive(iic_dev, sbuff, sbuff_len, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t touch_hi8561_i2c_write(esp_lcd_touch_handle_t tp, uint8_t *buff, uint8_t buff_len)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    // *INDENT-OFF*
    /* Write data */
    i2c_master_dev_handle_t iic_dev = (i2c_master_dev_handle_t)tp->config.user_data;
    return i2c_master_transmit(iic_dev, buff, buff_len, pdMS_TO_TICKS(1000));
    // *INDENT-ON*
}
