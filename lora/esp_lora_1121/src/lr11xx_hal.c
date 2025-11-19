/*!
 * @file      lr11xx_hal.c
 *
 * @brief     Hardware Abstraction Layer (HAL) implementation for lr1121
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdint.h>
#include "esp_lora_1121.h"

/*!
 * @brief lr11xx_hal.h API implementation
 */

/*!
 * @brief Function to wait the that lr1121 modem-e busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1121_hal_status_t
 */
static lr11xx_hal_status_t lr11xx_hal_wait_on_unbusy(const void *context, uint32_t timeout_ms);

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length)
{
#if defined(USE_LR11XX_CRC_OVER_SPI)
    uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
    if (data_length > 0){
        cmd_crc = lr11xx_hal_compute_crc(cmd_crc, data, data_length);
    }
        
#endif

    if (lr11xx_hal_wait_on_unbusy(context, 10000) == LR11XX_HAL_STATUS_OK)
    {
        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        /* Send CMD */
        lora_spi_write_bytes(context, (uint8_t *)command, command_length);
        /* Send Data */
        if (data_length > 0)
        {
            lora_spi_write_bytes(context, (uint8_t *)data, data_length);
        }
#if defined(USE_LR11XX_CRC_OVER_SPI)
        lora_spi_write_bytes(context, &cmd_crc, 1);
#endif
        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        return LR11XX_HAL_STATUS_OK;
    }
    return LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length)
{
#if defined(USE_LR11XX_CRC_OVER_SPI)
    const uint8_t cmd_crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
#endif
    uint8_t                     dummy_byte_rx  = LR11XX_NOP;

    if (lr11xx_hal_wait_on_unbusy(context, 10000) == LR11XX_HAL_STATUS_OK)
    {
        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        /* Send CMD */
        // uint8_t rx_data[16] = {0};
        lora_spi_write_bytes(context, (uint8_t *)command, command_length);
#if defined(USE_LR11XX_CRC_OVER_SPI)
        lora_spi_write_bytes(context, &cmd_crc, 1);
#endif
        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        /* Wait on busy pin up to 1000 ms */
        if (lr11xx_hal_wait_on_unbusy(context, 1000) != LR11XX_HAL_STATUS_OK)
        {
            return LR11XX_HAL_STATUS_ERROR;
        }

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);

        /* dummy read */
        lora_spi_read_bytes(context, &dummy_byte_rx, 1);
        lora_spi_read_bytes(context, data, data_length);

#if defined(USE_LR11XX_CRC_OVER_SPI)
        uint8_t crc_rx;
        lora_spi_read_bytes(context, &crc_rx, 1);
#endif
        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

#if defined( USE_LR11XX_CRC_OVER_SPI )
        uint8_t crc_computed = lr11xx_hal_compute_crc( 0xFF, &dummy_byte_rx, 1 );
        if(data_length > 0)
        {
            crc_computed         = lr11xx_hal_compute_crc( crc_computed, data, data_length );
        }
            
            if( crc_rx != crc_computed )
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
#endif
        return LR11XX_HAL_STATUS_OK;
    }
    return LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
                                           const uint16_t data_length)
{
    if (lr11xx_hal_wait_on_unbusy(context, 10000) == LR11XX_HAL_STATUS_OK)
    {
        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);

        lora_spi_read_bytes(context, data, data_length);

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        return LR11XX_HAL_STATUS_OK;
    }
    return LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{

    gpio_set_level(((lr1121_t *)context)->reset, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(((lr1121_t *)context)->reset, 1);

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
    /* Wakeup radio */
    gpio_set_level(((lr1121_t *)context)->cs, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(((lr1121_t *)context)->cs, 1);

    /* Wait on busy pin for 1000 ms */
    return LR11XX_HAL_STATUS_OK;
}

static lr11xx_hal_status_t lr11xx_hal_wait_on_unbusy(const void *context, uint32_t timeout_ms)
{
#if 0
     while( gpio_get_level( ( ( lr1121_t* ) context )->busy ) == 1 )
     {
         ;
     }
#else
    uint32_t start = esp_timer_get_time() / 1000;
    uint32_t current = 0;
    while (gpio_get_level(((lr1121_t *)context)->busy) == 1)
    {

        current = esp_timer_get_time() / 1000;
        if ((int32_t)(current - start) > (int32_t)timeout_ms)
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR11XX_HAL_STATUS_OK;
}
