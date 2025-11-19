/*!
 * @file      lr1121_modem_relay.c
 *
 * @brief     Relay driver implementation for LR1121 modem
 *
 * @copyright Copyright Semtech Corporation 2024. All rights reserved.
 *
 * @license{The Clear BSD License
 * @par
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
 * @par
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
 * POSSIBILITY OF SUCH DAMAGE.}
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1121_modem_relay.h"
#include <stdint.h>
#include "lr1121_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR1121_MODEM_RELAY_GET_TX_CONFIG_CMD_LENGTH ( 3 )
#define LR1121_MODEM_RELAY_SET_TX_CONFIG_RBUFFER_LENGTH ( 3 + 14 )

#define LR1121_MODEM_RELAY_GET_TX_CONFIG_RBUFFER_LENGTH ( 14 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operation code command
 */
enum
{
    LR1121_MODEM_RELAY_GET_TX_CONFIG_CMD = 0x00,
    LR1121_MODEM_RELAY_SET_TX_CONFIG_CMD = 0x01,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint32_t lr1121_uint8_to_uint32( const uint8_t value[4] );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1121_modem_response_code_t lr1121_modem_relay_get_tx_config( const void*                            context,
                                                               lr1121_modem_relay_tx_configuration_t* configuration )
{
    uint8_t rbuffer[LR1121_MODEM_RELAY_GET_TX_CONFIG_RBUFFER_LENGTH] = { 0 };

    const uint8_t cbuffer[LR1121_MODEM_RELAY_GET_TX_CONFIG_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_RELAY >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_RELAY,
        LR1121_MODEM_RELAY_GET_TX_CONFIG_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_RELAY_GET_TX_CONFIG_CMD_LENGTH, rbuffer,
        LR1121_MODEM_RELAY_GET_TX_CONFIG_RBUFFER_LENGTH );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        configuration->wor_second_channel_frequency_hz        = lr1121_uint8_to_uint32( rbuffer + 0 );
        configuration->wor_ack_second_channel_frequency_hz    = lr1121_uint8_to_uint32( rbuffer + 4 );
        configuration->wor_second_channel_datarate            = rbuffer[8];
        configuration->wor_second_channel_enable              = rbuffer[9];
        configuration->backoff_behavior                       = rbuffer[10];
        configuration->activation                             = ( lr1121_modem_relay_activation_t ) rbuffer[11];
        configuration->smart_level                            = ( lr1121_modem_relay_smart_level_t ) rbuffer[12];
        configuration->missed_ack_to_unsynchronized_threshold = rbuffer[13];
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_relay_set_tx_config(
    const void* context, const lr1121_modem_relay_tx_configuration_t* configuration )
{
    const uint8_t cbuffer[LR1121_MODEM_RELAY_SET_TX_CONFIG_RBUFFER_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_RELAY >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_RELAY,
        LR1121_MODEM_RELAY_SET_TX_CONFIG_CMD,
        ( uint8_t )( configuration->wor_second_channel_frequency_hz >> 24 ),
        ( uint8_t )( configuration->wor_second_channel_frequency_hz >> 16 ),
        ( uint8_t )( configuration->wor_second_channel_frequency_hz >> 8 ),
        ( uint8_t ) configuration->wor_second_channel_frequency_hz,
        ( uint8_t )( configuration->wor_ack_second_channel_frequency_hz >> 24 ),
        ( uint8_t )( configuration->wor_ack_second_channel_frequency_hz >> 16 ),
        ( uint8_t )( configuration->wor_ack_second_channel_frequency_hz >> 8 ),
        ( uint8_t ) configuration->wor_ack_second_channel_frequency_hz,
        configuration->wor_second_channel_datarate,
        configuration->wor_second_channel_enable,
        configuration->backoff_behavior,
        ( uint8_t ) configuration->activation,
        ( uint8_t ) configuration->smart_level,
        configuration->missed_ack_to_unsynchronized_threshold,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_RELAY_SET_TX_CONFIG_RBUFFER_LENGTH, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t lr1121_uint8_to_uint32( const uint8_t value[4] )
{
    return ( ( ( uint32_t ) value[0] ) << 24 ) + ( ( ( uint32_t ) value[1] ) << 16 ) +
           ( ( ( uint32_t ) value[2] ) << 8 ) + ( ( ( uint32_t ) value[3] ) );
}

/* --- EOF ------------------------------------------------------------------ */
