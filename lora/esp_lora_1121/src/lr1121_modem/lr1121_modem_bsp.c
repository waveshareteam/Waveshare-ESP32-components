/*!
 * @file      lr1121_modem_bsp.c
 *
 * @brief     BSP driver implementation for LR1121 modem
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
#include "lr1121_modem_bsp.h"
#include "lr1121_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * @brief Power config block length
 */
#define LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ( 5 )

#define LR1121_MODEM_TX_POWER_CONSUMPTION_SIZE_BYTE ( 5 )

#define LR1121_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_RF_OUTPUT_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_RF_OUTPUT_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH ( 3 + 4 )
#define LR1121_MODEM_GET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH ( 3 + 2 )
#define LR1121_MODEM_GET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH ( 3 + 8 )
#define LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH ( 3 )

#define LR1121_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH \
    ( LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS * LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH )
#define LR1121_MODEM_GET_TX_POWER_CONSUMPTION_RBUFFER_LENGTH \
    ( LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS * LR1121_MODEM_TX_POWER_CONSUMPTION_SIZE_BYTE )
#define LR1121_MODEM_GET_RX_POWER_CONSUMPTION_RBUFFER_LENGTH ( 8 )

#define LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA_DBUFFER_MAX_LENGTH \
    ( LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS * LR1121_MODEM_TX_POWER_CONSUMPTION_SIZE_BYTE )
#define LR1121_MODEM_SET_TX_OUTPUT_POWER_CONFIGURATION_DBUFFER_MAX_LENGTH \
    ( LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH * LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH )

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
    LR1121_MODEM_GET_TX_POWER_OFFSET_CMD          = 0x00,
    LR1121_MODEM_SET_TX_POWER_OFFSET_CMD          = 0x01,
    LR1121_MODEM_GET_OUTPUT_POWER_CONFIG_CMD      = 0x02,
    LR1121_MODEM_SET_OUTPUT_POWER_CONFIG_CMD      = 0x03,
    LR1121_MODEM_GET_RF_OUTPUT_CMD                = 0x04,
    LR1121_MODEM_SET_RF_OUTPUT_CMD                = 0x05,
    LR1121_MODEM_GET_CRYSTAL_ERROR_CMD            = 0x06,
    LR1121_MODEM_SET_CRYSTAL_ERROR_CMD            = 0x07,
    LR1121_MODEM_GET_XOSC_CAPA_TRIM_A_B           = 0x08,
    LR1121_MODEM_SET_XOSC_CAPA_TRIM_A_B           = 0x09,
    LR1121_MODEM_GET_TX_POWER_CONSUMPTION_UA      = 0x0A,
    LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA      = 0x0B,
    LR1121_MODEM_GET_LORA_RX_POWER_CONSUMPTION_UA = 0x0C,
    LR1121_MODEM_SET_LORA_RX_POWER_CONSUMPTION_UA = 0x0D,
    LR1121_MODEM_GET_GFSK_RX_POWER_CONSUMPTION_UA = 0x0E,
    LR1121_MODEM_SET_GFSK_RX_POWER_CONSUMPTION_UA = 0x0F,
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

static lr1121_modem_response_code_t lr1121_modem_get_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption, uint8_t opcode );

static lr1121_modem_response_code_t lr1121_modem_set_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption, uint8_t opcode );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1121_modem_response_code_t lr1121_modem_get_tx_power_offset( const void* context, int8_t* tx_power_offset )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_GET_TX_POWER_OFFSET_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH, ( uint8_t* ) tx_power_offset, sizeof( int8_t ) );
}

lr1121_modem_response_code_t lr1121_modem_set_tx_power_offset( const void* context, const int8_t tx_power_offset )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_SET_TX_POWER_OFFSET_CMD,
        ( uint8_t ) tx_power_offset,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_set_output_power_config(
    const void* context, const lr1121_modem_output_power_config_t* output_power_configs,
    uint8_t n_output_power_configs )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_SET_OUTPUT_POWER_CONFIG_CMD,
    };

    // Array of maximal size is allocated
    uint8_t dbuffer[LR1121_MODEM_SET_TX_OUTPUT_POWER_CONFIGURATION_DBUFFER_MAX_LENGTH] = { 0 };

    for( uint8_t index_power_config = 0; index_power_config < n_output_power_configs; index_power_config++ )
    {
        const lr1121_modem_output_power_config_t local_power_config = output_power_configs[index_power_config];
        const uint8_t local_dbuffer_index = index_power_config * LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH;

        dbuffer[local_dbuffer_index]     = local_power_config.expected_power;
        dbuffer[local_dbuffer_index + 1] = local_power_config.configured_power;
        dbuffer[local_dbuffer_index + 2] =
            ( uint8_t )( ( local_power_config.pa_supply & 0x0F ) | ( local_power_config.pa_sel << 4 ) );
        dbuffer[local_dbuffer_index + 3] =
            ( ( local_power_config.pa_duty_cycle & 0x0F ) << 4 ) | ( local_power_config.pa_hp_sel & 0x0F );
        dbuffer[local_dbuffer_index + 4] = ( uint8_t ) local_power_config.pa_ramp_time;
    }

    const uint8_t dbuffer_size = n_output_power_configs * LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH;
    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH, dbuffer, dbuffer_size );
}

lr1121_modem_response_code_t lr1121_modem_get_output_power_config(
    const void* context, lr1121_modem_output_power_config_list_t output_power_config )
{
    uint8_t rbuffer[LR1121_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_GET_OUTPUT_POWER_CONFIG_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH, rbuffer,
        LR1121_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH );

    for( uint8_t i = 0; i < LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS; i++ )
    {
        const uint8_t local_rbuffer_index = i * LR1121_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH;

        output_power_config[i].expected_power   = rbuffer[local_rbuffer_index];
        output_power_config[i].configured_power = rbuffer[local_rbuffer_index + 1];
        output_power_config[i].pa_supply = ( lr1121_modem_pa_reg_supply_t )( rbuffer[local_rbuffer_index + 2] & 0x0F );
        output_power_config[i].pa_sel =
            ( lr1121_modem_output_power_configuration_pa_sel_t )( ( rbuffer[local_rbuffer_index + 2] >> 4 ) & 0x0F );
        output_power_config[i].pa_duty_cycle = ( rbuffer[local_rbuffer_index + 3] & 0xF0 ) >> 4;
        output_power_config[i].pa_hp_sel     = rbuffer[local_rbuffer_index + 3] & 0x0F;
        output_power_config[i].pa_ramp_time  = ( lr1121_modem_ramp_time_t )( rbuffer[local_rbuffer_index + 4] );
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_rf_output( const void*                                 context,
                                                         const lr1121_modem_bsp_radio_pa_selection_t output )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_RF_OUTPUT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_SET_RF_OUTPUT_CMD,
        ( uint8_t ) output,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_SET_RF_OUTPUT_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_rf_output( const void*                                  context,
                                                         const lr1121_modem_bsp_radio_pa_selection_t* output )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_RF_OUTPUT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_GET_RF_OUTPUT_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_RF_OUTPUT_CMD_LENGTH, ( uint8_t* ) output, sizeof( uint8_t ) );
}

lr1121_modem_response_code_t lr1121_modem_get_crystal_error( const void* context, uint32_t* crystal_error_ppm )
{
    uint8_t rbuffer[sizeof( uint32_t )] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_GET_CRYSTAL_ERROR_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *crystal_error_ppm = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                             ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_crystal_error( const void* context, const uint32_t crystal_error_ppm )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_SET_CRYSTAL_ERROR_CMD,
        ( uint8_t )( crystal_error_ppm >> 24 ),
        ( uint8_t )( crystal_error_ppm >> 16 ),
        ( uint8_t )( crystal_error_ppm >> 8 ),
        ( uint8_t ) crystal_error_ppm,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_xosc_capa_trim_a_b( const void* context, uint8_t* capa_trim_a,
                                                                  uint8_t* capa_trim_b )
{
    uint8_t rbuffer[sizeof( uint16_t )] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_GET_XOSC_CAPA_TRIM_A_B,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );
    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *capa_trim_a = rbuffer[0];
        *capa_trim_b = rbuffer[1];
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_xosc_capa_trim_a_b( const void* context, const uint8_t capa_trim_a,
                                                                  const uint8_t capa_trim_b )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_BSP >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_BSP,
        LR1121_MODEM_SET_XOSC_CAPA_TRIM_A_B,
        capa_trim_a,
        capa_trim_b,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_SET_XOSC_CAPA_TRIM_A_B_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_tx_power_consumption_ua(
    const void* context, lr1121_modem_tx_power_consumption_list_t consumption_per_power )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH] = {
        ( LR1121_MODEM_GROUP_ID_BSP >> 8 ) & 0xFF,
        LR1121_MODEM_GROUP_ID_BSP & 0xFF,
        LR1121_MODEM_GET_TX_POWER_CONSUMPTION_UA,
    };

    uint8_t rbuffer[LR1121_MODEM_GET_TX_POWER_CONSUMPTION_RBUFFER_LENGTH] = { 0 };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH, rbuffer,
        LR1121_MODEM_GET_TX_POWER_CONSUMPTION_RBUFFER_LENGTH );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        for( uint8_t index_power = 0; index_power < LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS; index_power++ )
        {
            lr1121_modem_tx_power_consumption_value_t* local_value = &consumption_per_power[index_power];
            local_value->tx_power_dbm                              = rbuffer[index_power * 5];
            local_value->consumed_power_ua =
                ( rbuffer[index_power * 5 + 1] << 24 ) + ( rbuffer[index_power * 5 + 2] << 16 ) +
                ( rbuffer[index_power * 5 + 3] << 8 ) + ( rbuffer[index_power * 5 + 4] << 0 );
        }
    }
    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_tx_power_consumption_ua(
    const void* context, const lr1121_modem_tx_power_consumption_value_t* consumption_per_power,
    uint8_t n_consumption_per_power )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH] = {
        ( LR1121_MODEM_GROUP_ID_BSP >> 8 ) & 0xFF,
        LR1121_MODEM_GROUP_ID_BSP & 0xFF,
        LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA,
    };

    // Array of maximal size is allocated
    uint8_t dbuffer[LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA_DBUFFER_MAX_LENGTH] = { 0 };

    for( uint8_t power_table_index = 0; power_table_index < n_consumption_per_power; power_table_index++ )
    {
        const lr1121_modem_tx_power_consumption_value_t local_consumption_value =
            consumption_per_power[power_table_index];
        const uint8_t local_dbuffer_index = power_table_index * LR1121_MODEM_TX_POWER_CONSUMPTION_SIZE_BYTE;

        dbuffer[local_dbuffer_index]     = local_consumption_value.tx_power_dbm;
        dbuffer[local_dbuffer_index + 1] = ( uint8_t )( local_consumption_value.consumed_power_ua >> 24 );
        dbuffer[local_dbuffer_index + 2] = ( uint8_t )( local_consumption_value.consumed_power_ua >> 16 );
        dbuffer[local_dbuffer_index + 3] = ( uint8_t )( local_consumption_value.consumed_power_ua >> 8 );
        dbuffer[local_dbuffer_index + 4] = ( uint8_t ) local_consumption_value.consumed_power_ua;
    }

    // Number of meaningful bytes from dbuffer to transmit to the chip
    const uint8_t dbuffer_size = n_consumption_per_power * LR1121_MODEM_TX_POWER_CONSUMPTION_SIZE_BYTE;

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_SET_TX_POWER_CONSUMPTION_UA_CMD_LENGTH, dbuffer, dbuffer_size );
}

lr1121_modem_response_code_t lr1121_modem_get_lora_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption )
{
    return lr1121_modem_get_rx_power_consumption_ua( context, rx_consumption,
                                                     LR1121_MODEM_GET_LORA_RX_POWER_CONSUMPTION_UA );
}

lr1121_modem_response_code_t lr1121_modem_set_lora_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption )
{
    return lr1121_modem_set_rx_power_consumption_ua( context, rx_consumption,
                                                     LR1121_MODEM_SET_LORA_RX_POWER_CONSUMPTION_UA );
}

lr1121_modem_response_code_t lr1121_modem_get_gfsk_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption )
{
    return lr1121_modem_get_rx_power_consumption_ua( context, rx_consumption,
                                                     LR1121_MODEM_GET_GFSK_RX_POWER_CONSUMPTION_UA );
}

lr1121_modem_response_code_t lr1121_modem_set_gfsk_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption )
{
    return lr1121_modem_set_rx_power_consumption_ua( context, rx_consumption,
                                                     LR1121_MODEM_SET_GFSK_RX_POWER_CONSUMPTION_UA );
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

lr1121_modem_response_code_t lr1121_modem_get_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption, uint8_t opcode )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH] = {
        ( LR1121_MODEM_GROUP_ID_BSP >> 8 ) & 0xFF,
        LR1121_MODEM_GROUP_ID_BSP & 0xFF,
        opcode,
    };

    uint8_t rbuffer[LR1121_MODEM_GET_RX_POWER_CONSUMPTION_RBUFFER_LENGTH] = { 0 };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH, rbuffer,
        LR1121_MODEM_GET_RX_POWER_CONSUMPTION_RBUFFER_LENGTH );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        rx_consumption->consumption_rx_boosted_off_ua = lr1121_uint8_to_uint32( rbuffer );
        rx_consumption->consumption_rx_boosted_on_ua  = lr1121_uint8_to_uint32( rbuffer + 4 );
    }
    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption, uint8_t opcode )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH] = {
        ( LR1121_MODEM_GROUP_ID_BSP >> 8 ) & 0xFF,
        LR1121_MODEM_GROUP_ID_BSP & 0xFF,
        opcode,
        ( uint8_t )( rx_consumption->consumption_rx_boosted_off_ua >> 24 ),
        ( uint8_t )( rx_consumption->consumption_rx_boosted_off_ua >> 16 ),
        ( uint8_t )( rx_consumption->consumption_rx_boosted_off_ua >> 8 ),
        ( uint8_t ) rx_consumption->consumption_rx_boosted_off_ua,
        ( uint8_t )( rx_consumption->consumption_rx_boosted_on_ua >> 24 ),
        ( uint8_t )( rx_consumption->consumption_rx_boosted_on_ua >> 16 ),
        ( uint8_t )( rx_consumption->consumption_rx_boosted_on_ua >> 8 ),
        ( uint8_t ) rx_consumption->consumption_rx_boosted_on_ua,
    };
    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_SET_RX_POWER_CONSUMPTION_UA_CMD_LENGTH, 0, 0 );
}

/* --- EOF ------------------------------------------------------------------ */
