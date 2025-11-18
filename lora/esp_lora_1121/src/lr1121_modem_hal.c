/*!
 * @file      lr1121_modem_hal.c
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
 * @brief lr1121_modem_hal.h API implementation
 */

/*!
 * @brief Function to wait that the lr1121 modem-e busy line fall to low
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1121_hal_status_t
 */
static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_busy(const void *context, uint32_t timeout_ms);

/*!
 * @brief Function to wait the that lr1121 modem-e busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1121_hal_status_t
 */
static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_unbusy(const void *context, uint32_t timeout_ms);

lr1121_modem_hal_status_t lr1121_modem_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length)
{

    if (lr1121_modem_hal_wakeup( context ) == LR1121_MODEM_HAL_STATUS_OK)
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1121_modem_hal_status_t status;

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        /* Send CMD */
        lora_spi_write_bytes(context, (uint8_t *)command, command_length);
        /* Send Data */
        if (data_length > 0)
        {
            lora_spi_write_bytes(context, (uint8_t *)data, data_length);
        }

        /* Compute and send CRC */
        crc = lr1121_modem_compute_crc( 0xFF, command, command_length );
        if (data_length > 0)
        {
            crc = lr1121_modem_compute_crc( crc, data, data_length );
        }    
        /* Send CRC */
        lora_spi_write_bytes(context, (uint8_t *)&crc, 1);

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        /* Wait on busy pin up to 1000 ms */
        if( lr1121_modem_hal_wait_on_busy( context, 1000 ) != LR1121_MODEM_HAL_STATUS_OK )
        {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        /* Send dummy byte to retrieve RC & CRC */

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);

        /* read RC */
        lora_spi_read_bytes(context, ( uint8_t* ) &status, 1);
        lora_spi_read_bytes(context, ( uint8_t* ) &crc_received, 1);

        /* Compute response crc */
        crc = lr1121_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        if( crc != crc_received )
        {
            /* change the response code */
            status = LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if( lr1121_modem_hal_wait_on_unbusy( context, 1000 ) != LR1121_MODEM_HAL_STATUS_OK )
        {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }
    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1121_modem_hal_status_t lr1121_modem_hal_write_without_rc( const void* context, const uint8_t* command,
    const uint16_t command_length, const uint8_t* data,
    const uint16_t data_length )
{
    if( lr1121_modem_hal_wakeup( context ) == LR1121_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc         = 0;
        lr1121_modem_hal_status_t status      = LR1121_MODEM_HAL_STATUS_OK;

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        /* Send CMD */
        lora_spi_write_bytes(context, (uint8_t *)command, command_length);
        /* Send Data */
        if (data_length > 0)
        {
            lora_spi_write_bytes(context, (uint8_t *)data, data_length);
        }
        /* Compute and send CRC */
        crc = lr1121_modem_compute_crc( 0xFF, command, command_length );
        crc = lr1121_modem_compute_crc( crc, data, data_length );
        /* Send CRC */
        lora_spi_write_bytes(context, (uint8_t *)&crc, 1);
        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        return status;
    }
    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1121_modem_hal_status_t lr1121_modem_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length)
{
    if (lr1121_modem_hal_wakeup( context ) == LR1121_MODEM_HAL_STATUS_OK)
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1121_modem_hal_status_t status = LR1121_MODEM_HAL_STATUS_ERROR;

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        /* Send CMD */
        lora_spi_write_bytes(context, (uint8_t *)command, command_length);

        /* Compute and send CRC */
        crc = lr1121_modem_compute_crc( 0xFF, command, command_length );
        /* Send CRC */
        lora_spi_write_bytes(context, &crc, 1);

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        /* Wait on busy pin up to 1000 ms */
        if (lr1121_modem_hal_wait_on_busy(context, 1000) != LR1121_MODEM_HAL_STATUS_OK)
        {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }

        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);

        /* read RC */
        lora_spi_read_bytes(context, ( uint8_t* ) &status, 1);
        // printf("status:%d\r\n",status);
        if( status == LR1121_MODEM_HAL_STATUS_OK )
        {
            // printf("data_length:%d\r\n",data_length);
            lora_spi_read_bytes(context, data, data_length);
        }
        lora_spi_read_bytes( context, ( uint8_t* ) &crc_received, 1 );

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        /* Compute response crc */
        crc = lr1121_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );
        if( status == LR1121_MODEM_HAL_STATUS_OK )
        {
            crc = lr1121_modem_compute_crc( crc, data, data_length );
        }

        if( crc != crc_received )
        {
        if( crc != crc_received )
            /* change the response code */
            status = LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if( lr1121_modem_hal_wait_on_unbusy( context, 1000 ) != LR1121_MODEM_HAL_STATUS_OK )
        {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }
    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1121_modem_hal_status_t lr1121_modem_hal_direct_read(const void *context, uint8_t *data,
                                           const uint16_t data_length)
{
    if (lr1121_modem_hal_wait_on_unbusy(context, 10000) == LR1121_MODEM_HAL_STATUS_OK)
    {
        /* NSS low */
        gpio_set_level(((lr1121_t *)context)->cs, 0);

        lora_spi_read_bytes(context, data, data_length);

        /* NSS high */
        gpio_set_level(((lr1121_t *)context)->cs, 1);

        return LR1121_MODEM_HAL_STATUS_OK;
    }
    return LR1121_MODEM_HAL_STATUS_ERROR;
}

lr1121_modem_hal_status_t lr1121_modem_hal_reset(const void *context)
{

    gpio_set_level(((lr1121_t *)context)->reset, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(((lr1121_t *)context)->reset, 1);

    return LR1121_MODEM_HAL_STATUS_OK;
}

lr1121_modem_hal_status_t lr1121_modem_hal_wakeup(const void *context)
{
    if( lr1121_modem_hal_wait_on_busy( context, 10000 ) == LR1121_MODEM_HAL_STATUS_OK )
    {
        /* Wakeup radio */
        gpio_set_level(((lr1121_t *)context)->cs, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(((lr1121_t *)context)->cs, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    else
    {
        return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Wait on busy pin for 1000 ms */
    return lr1121_modem_hal_wait_on_unbusy( context, 1000 );
}


static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_busy(const void *context, uint32_t timeout_ms)
{
#if 0
     while( gpio_get_level( ( ( lr1121_t* ) context )->busy ) == 0 )
     {
         ;
     }
#else
    uint32_t start = esp_timer_get_time() / 1000;
    uint32_t current = 0;

    while (gpio_get_level(((lr1121_t *)context)->busy) == 0)
    {
        current = esp_timer_get_time() / 1000;
        if ((int32_t)(current - start) > (int32_t)timeout_ms)
        {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }
    }

#endif
    return LR1121_MODEM_HAL_STATUS_OK;
}

static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_unbusy(const void *context, uint32_t timeout_ms)
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
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1121_MODEM_HAL_STATUS_OK;
}
