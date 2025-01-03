/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "bsp/esp32_s3_touch_amoled_1_8_bsp.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "ESP32-S3-Touch-AMOLED-1.8-BSP";

/* Sample rate of MSM261S4030H0 */
#define BSP_MIC_SAMPLE_RATE (48000u)

/* This configuration is used by default in bsp_audio_init() */
#define BSP_I2S_SIMPLEX_MONO_CFG(_sample_rate)                      \
    {                                                                 \
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,          \
        .sample_rate = _sample_rate,                                  \
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                 \
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                  \
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,            \
        .dma_buf_count = 3,                                           \
        .dma_buf_len = 1024,                                          \
        .use_apll = true,                                             \
        .tx_desc_auto_clear = true,                                   \
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM \
    }


esp_err_t bsp_adc_initialize(void)
{
    return ESP_OK;
}
