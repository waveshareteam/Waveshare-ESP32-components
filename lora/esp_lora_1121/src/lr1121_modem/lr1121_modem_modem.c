/*!
 * @file      lr1121_modem_modem.c
 *
 * @brief     modem driver implementation for LR1121 modem
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
#include "lr1121_modem_modem.h"
#include "lr1121_modem_common.h"
#include "lr1121_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1121_MODEM_FACTORY_RESET_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_VERSION_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_STATUS_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_CHARGE_CMD_LENGTH ( 3 )
#define LR1121_MODEM_RESET_CHARGE_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_EVENT_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_SUSPEND_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_GET_SUSPEND_CMD_LENGTH ( 3 )
#define LR1121_MODEM_SET_ALARM_TIMER_CMD_LENGTH ( 3 + 4 )
#define LR1121_MODEM_CLEAR_ALARM_TIMER_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_ALARM_REMAINING_TIME_CMD_LENGTH ( 3 )
#define LR1121_MODEM_GET_CRASHLOG_CMD_LENGTH ( 3 )

#define LR1121_MODEM_TEST_MODE_TST_START_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_TEST_MODE_TST_TX_LORA_CMD_LENGTH ( 3 + 25 )
#define LR1121_MODEM_TEST_MODE_TST_TX_FSK_CMD_LENGTH ( 3 + 15 )
#define LR1121_MODEM_TEST_MODE_TST_TX_LR_FHSS_CMD_LENGTH ( 3 + 19 )
#define LR1121_MODEM_TEST_MODE_TST_TX_CONT_CMD_LENGTH ( 3 + 10 )
#define LR1121_MODEM_TEST_MODE_TST_CW_CMD_LENGTH ( 3 + 6 )
#define LR1121_MODEM_TEST_MODE_TST_RX_LORA_CONT_CMD_LENGTH ( 3 + 8 )
#define LR1121_MODEM_TEST_MODE_TST_RX_FSK_CONT_CMD_LENGTH ( 3 + 5 )
#define LR1121_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH ( 3 + 11 )
#define LR1121_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH ( 3 + 1 )
#define LR1121_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM_CMD_LENGTH ( 3 + 12 )
#define LR1121_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH ( 3 + 1 )

#define LR1121_MODEM_GET_CHARGE_RBUFFER_LENGTH ( 320 )
#define LR1121_MODEM_GET_CRASHLOG_RBUFFER_LENGTH ( 243 )
#define LR1121_MODEM_GET_VERSION_RBUFFER_LENGTH ( 9 )
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operation code command
 */
enum
{
    LR1121_MODEM_FACTORY_RESET_CMD            = 0x00,
    LR1121_MODEM_GET_VERSION_CMD              = 0x01,
    LR1121_MODEM_GET_STATUS_CMD               = 0x02,
    LR1121_MODEM_GET_CHARGE_CMD               = 0x03,
    LR1121_MODEM_GET_EVENT_CMD                = 0x04,
    LR1121_MODEM_TEST_CMD                     = 0x05,
    LR1121_MODEM_GET_SUSPEND_MODEM_COM_CMD    = 0x06,
    LR1121_MODEM_SET_SUSPEND_MODEM_COM_CMD    = 0x07,
    LR1121_MODEM_SET_ALARM_TIMER_CMD          = 0x08,
    LR1121_MODEM_CLEAR_ALARM_TIMER_CMD        = 0x09,
    LR1121_MODEM_GET_ALARM_REMAINING_TIME_CMD = 0x0A,
    LR1121_MODEM_GET_CRASHLOG_CMD             = 0x0B,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Radio test mode type
 */
typedef enum
{
    LR1121_MODEM_TEST_MODE_TST_START                       = 0x00,
    LR1121_MODEM_TEST_MODE_TST_EXIT                        = 0x01,
    LR1121_MODEM_TEST_MODE_TST_NOP                         = 0x02,
    LR1121_MODEM_TEST_MODE_TST_TX_LORA                     = 0x03,
    LR1121_MODEM_TEST_MODE_TST_TX_FSK                      = 0x04,
    LR1121_MODEM_TEST_MODE_TST_TX_LR_FHSS                  = 0x05,
    LR1121_MODEM_TEST_MODE_TST_TX_CW                       = 0x06,
    LR1121_MODEM_TEST_MODE_TST_RX_LORA_CONT                = 0x07,
    LR1121_MODEM_TEST_MODE_TST_RX_FSK_CONT                 = 0x08,
    LR1121_MODEM_TEST_MODE_TST_READ_RX_PKT_COUNTER_RX_CONT = 0x09,
    LR1121_MODEM_TEST_MODE_TST_RSSI_SUBGHZ                 = 0x0A,
    LR1121_MODEM_TEST_MODE_READ_RSSI                       = 0x0B,
    LR1121_MODEM_TEST_MODE_TST_RADIO_RST                   = 0x0E,
    LR1121_MODEM_TEST_MODE_TST_READ_REGISTER               = 0x0F,
    LR1121_MODEM_TEST_MODE_TST_WRITE_REGISTER              = 0x10,
} lr1121_modem_test_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Helper function that convert an array of uint8_t into a uint32_t single value
 *
 * @warning It is up to the caller to ensure that value points to an array of at least sizeof(uint32_t) elements.
 *
 * @param [in] value Array of uint8_t to be translated into a uint32_t
 *
 * @returns 32-bit value
 */
static uint32_t lr1121_uint8_to_uint32( const uint8_t value[4] );

static void lr1121_parse_charge_hook_id( const uint8_t* buffer, uint8_t hook_id, lr1121_modem_consumption_details_t* hook );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1121_modem_response_code_t lr1121_modem_factory_reset( const void* context )
{
    const uint8_t cbuffer[LR1121_MODEM_FACTORY_RESET_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_FACTORY_RESET_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write_without_rc(
        context, cbuffer, LR1121_MODEM_FACTORY_RESET_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_modem_version( const void*             context,
                                                             lr1121_modem_version_t* modem_version )
{
    uint8_t rbuffer[LR1121_MODEM_GET_VERSION_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_VERSION_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_VERSION_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_VERSION_CMD_LENGTH, rbuffer, LR1121_MODEM_GET_VERSION_RBUFFER_LENGTH );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        modem_version->use_case    = rbuffer[0];
        modem_version->modem_major = rbuffer[1];
        modem_version->modem_minor = rbuffer[2];
        modem_version->modem_patch = rbuffer[3];
        modem_version->lbm_major   = rbuffer[5];
        modem_version->lbm_minor   = rbuffer[6];
        modem_version->lbm_patch   = rbuffer[7];
    }
    return rc;
}

lr1121_modem_response_code_t lr1121_modem_get_status( const void*                            context,
                                                      lr1121_modem_lorawan_status_bitmask_t* status )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_STATUS_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_read( context, cbuffer, LR1121_MODEM_GET_STATUS_CMD_LENGTH,
                                                                   ( uint8_t* ) status, sizeof( uint8_t ) );
}

lr1121_modem_response_code_t lr1121_modem_get_charge( const void* context, lr1121_modem_charge_t* charge )
{
    uint8_t rbuffer[LR1121_MODEM_GET_CHARGE_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_CHARGE_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_CHARGE_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_CHARGE_CMD_LENGTH, rbuffer, LR1121_MODEM_GET_CHARGE_RBUFFER_LENGTH );

    lr1121_parse_charge_hook_id( rbuffer, 0, &charge->suspend );
    lr1121_parse_charge_hook_id( rbuffer, 1, &charge->class_b_beacon );
    lr1121_parse_charge_hook_id( rbuffer, 2, &charge->lr1mac_stack );
    lr1121_parse_charge_hook_id( rbuffer, 3, &charge->lbt );
    lr1121_parse_charge_hook_id( rbuffer, 4, &charge->cad );
    lr1121_parse_charge_hook_id( rbuffer, 5, &charge->class_b_ping_slot );
    lr1121_parse_charge_hook_id( rbuffer, 6, &charge->test_mode );
    lr1121_parse_charge_hook_id( rbuffer, 7, &charge->direct_rp_access );
    lr1121_parse_charge_hook_id( rbuffer, 8, &charge->relay_tx );
    lr1121_parse_charge_hook_id( rbuffer, 9, &charge->class_c );

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_get_event( const void* context, lr1121_modem_event_fields_t* event_fields )
{
    uint8_t rbuffer[sizeof( uint32_t )] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_EVENT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_EVENT_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_EVENT_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    event_fields->event_type          = ( lr1121_modem_lorawan_event_type_t ) rbuffer[0];
    event_fields->missed_events_count = rbuffer[1];
    event_fields->data                = ( ( uint16_t ) rbuffer[2] << 8 ) + ( ( uint16_t ) rbuffer[3] );

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_set_suspend( const void* context, const lr1121_modem_suspend_t suspend )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_SUSPEND_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_SET_SUSPEND_MODEM_COM_CMD,
        suspend,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_SET_SUSPEND_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_suspend( const void* context, lr1121_modem_suspend_t* suspend )
{
    const uint8_t cbuffer[LR1121_MODEM_GET_SUSPEND_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_SUSPEND_MODEM_COM_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_SUSPEND_CMD_LENGTH, ( uint8_t* ) suspend, sizeof( uint8_t ) );
}

lr1121_modem_response_code_t lr1121_modem_set_alarm_timer( const void* context, uint32_t seconds )
{
    const uint8_t cbuffer[LR1121_MODEM_SET_ALARM_TIMER_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_SET_ALARM_TIMER_CMD,
        ( uint8_t )( seconds >> 24 ),
        ( uint8_t )( seconds >> 16 ),
        ( uint8_t )( seconds >> 8 ),
        ( uint8_t ) seconds,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_SET_ALARM_TIMER_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_clear_alarm_timer( const void* context )
{
    const uint8_t cbuffer[LR1121_MODEM_CLEAR_ALARM_TIMER_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_CLEAR_ALARM_TIMER_CMD,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_CLEAR_ALARM_TIMER_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_get_alarm_remaining_time( const void* context, uint32_t* remaining_time )
{
    uint8_t rbuffer[sizeof( uint32_t )] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_ALARM_REMAINING_TIME_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_ALARM_REMAINING_TIME_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_ALARM_REMAINING_TIME_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *remaining_time = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                          ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_get_crashlog( const void* context, lr1121_modem_crashlog_status_t* status,
                                                        uint8_t* crashlog )
{
    uint8_t rbuffer[LR1121_MODEM_GET_CRASHLOG_RBUFFER_LENGTH] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_GET_CRASHLOG_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_GET_CRASHLOG_CMD,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_GET_CRASHLOG_CMD_LENGTH, rbuffer, LR1121_MODEM_GET_CRASHLOG_RBUFFER_LENGTH );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *status = ( lr1121_modem_crashlog_status_t ) rbuffer[0];

        for( uint8_t i = 0; i < LR1121_MODEM_GET_CRASHLOG_RBUFFER_LENGTH - 1; i++ )
        {
            crashlog[i] = rbuffer[1 + i];
        }
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_test_mode_start( const void* context )
{
    const uint8_t test_msg[8] = { 'T', 'E', 'S', 'T', 'T', 'E', 'S', 'T' };

    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_START_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_START,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_START_CMD_LENGTH, test_msg, 8 );
}

lr1121_modem_response_code_t lr1121_modem_test_exit( const void* context )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_EXIT,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_nop( const void* context )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_NOP,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_tx_lora(
    const void* context, uint32_t frequency, int8_t tx_power, uint8_t payload_length,
    lr1121_modem_tst_mode_lora_sf_t sf, lr1121_modem_tst_mode_lora_bw_t bw, lr1121_modem_tst_mode_lora_cr_t cr,
    bool is_iq_inverted, bool is_crc_enabled, lr1121_modem_tst_mode_lora_packet_header_mode_t header_mode,
    uint32_t preamble_length, uint32_t number_of_tx, uint32_t delay_ms )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_TX_LORA_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_TX_LORA,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t ) tx_power,
        payload_length,
        ( uint8_t ) sf,
        ( uint8_t ) bw,
        ( uint8_t ) cr,
        ( uint8_t )( is_iq_inverted ? 0x01 : 0x00 ),
        ( uint8_t )( is_crc_enabled ? 0x01 : 0x00 ),
        ( uint8_t ) header_mode,
        ( uint8_t )( preamble_length >> 24 ),
        ( uint8_t )( preamble_length >> 16 ),
        ( uint8_t )( preamble_length >> 8 ),
        ( uint8_t ) preamble_length,
        ( uint8_t )( number_of_tx >> 24 ),
        ( uint8_t )( number_of_tx >> 16 ),
        ( uint8_t )( number_of_tx >> 8 ),
        ( uint8_t ) number_of_tx,
        ( uint8_t )( delay_ms >> 24 ),
        ( uint8_t )( delay_ms >> 16 ),
        ( uint8_t )( delay_ms >> 8 ),
        ( uint8_t ) delay_ms,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_TX_LORA_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_tx_fsk( const void* context, uint32_t frequency, int8_t tx_power,
                                                       uint8_t payload_length, uint32_t number_of_tx,
                                                       uint32_t delay_ms )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_TX_FSK_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_TX_FSK,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t ) tx_power,
        payload_length,
        ( uint8_t )( number_of_tx >> 24 ),
        ( uint8_t )( number_of_tx >> 16 ),
        ( uint8_t )( number_of_tx >> 8 ),
        ( uint8_t ) number_of_tx,
        ( uint8_t )( delay_ms >> 24 ),
        ( uint8_t )( delay_ms >> 16 ),
        ( uint8_t )( delay_ms >> 8 ),
        ( uint8_t ) delay_ms,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_TX_FSK_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_tx_lr_fhss( const void* context, uint32_t frequency, int8_t tx_power,
                                                           uint8_t                              payload_length,
                                                           lr1121_modem_tst_mode_lr_fhss_grid_t grid,
                                                           lr1121_modem_tst_mode_lr_fhss_bw_t   bw,
                                                           lr1121_modem_tst_mode_lr_fhss_cr_t cr, uint32_t number_of_tx,
                                                           uint32_t delay_ms, bool is_hopping_enabled )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_TX_LR_FHSS_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_TX_LR_FHSS,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t ) tx_power,
        payload_length,
        ( uint8_t ) grid,
        ( uint8_t ) bw,
        ( uint8_t ) cr,
        ( uint8_t )( number_of_tx >> 24 ),
        ( uint8_t )( number_of_tx >> 16 ),
        ( uint8_t )( number_of_tx >> 8 ),
        ( uint8_t ) number_of_tx,
        ( uint8_t )( delay_ms >> 24 ),
        ( uint8_t )( delay_ms >> 16 ),
        ( uint8_t )( delay_ms >> 8 ),
        ( uint8_t ) delay_ms,
        is_hopping_enabled ? 0x01 : 0x00,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_TX_LR_FHSS_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_tx_cw( const void* context, uint32_t frequency, int8_t tx_power )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_CW_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_TX_CW,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t ) tx_power,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write( context, cbuffer,
                                                                    LR1121_MODEM_TEST_MODE_TST_CW_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_rx_lora_cont( const void* context, uint32_t frequency,
                                                             lr1121_modem_tst_mode_lora_sf_t sf,
                                                             lr1121_modem_tst_mode_lora_bw_t bw,
                                                             lr1121_modem_tst_mode_lora_cr_t cr )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_RX_LORA_CONT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_RX_LORA_CONT,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t ) sf,
        ( uint8_t ) bw,
        ( uint8_t ) cr,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_RX_LORA_CONT_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_rx_fsk_cont( const void* context, uint32_t frequency )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_RX_FSK_CONT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_RX_FSK_CONT,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_RX_FSK_CONT_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_read_packet_counter_rx_cont( const void* context,
                                                                            uint32_t*   rx_packet_counter )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_READ_RX_PKT_COUNTER_RX_CONT,
    };

    uint8_t rbuffer[sizeof( uint32_t )] = { 0 };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *rx_packet_counter = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                             ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_test_rssi_subghz( const void* context, uint32_t frequency, uint16_t time_ms,
                                                            uint32_t bw_hz )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_RSSI_SUBGHZ,
        ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( frequency & 0x000000FF ),
        ( uint8_t )( time_ms >> 8 ),
        ( uint8_t )( time_ms ),
        ( uint8_t )( ( bw_hz & 0xFF000000 ) >> 24 ),
        ( uint8_t )( ( bw_hz & 0x00FF0000 ) >> 16 ),
        ( uint8_t )( ( bw_hz & 0x0000FF00 ) >> 8 ),
        ( uint8_t )( bw_hz & 0x000000FF ),
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH, 0, 0 );
}

lr1121_modem_response_code_t lr1121_modem_test_read_rssi( const void* context, int16_t* rssi )
{
    uint8_t rbuffer[sizeof( uint8_t )] = { 0x00 };

    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_READ_RSSI,
    };

    const lr1121_modem_response_code_t rc = ( lr1121_modem_response_code_t ) lr1121_modem_hal_read(
        context, cbuffer, LR1121_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH, rbuffer, sizeof( uint8_t ) );

    if( rc == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        *rssi = ( int16_t )( ( int8_t ) rbuffer[0] - 64 );
    }

    return rc;
}

lr1121_modem_response_code_t lr1121_modem_test_radio_rst( const void* context )
{
    const uint8_t cbuffer[LR1121_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH] = {
        ( uint8_t )( LR1121_MODEM_GROUP_ID_MODEM >> 8 ),
        ( uint8_t ) LR1121_MODEM_GROUP_ID_MODEM,
        LR1121_MODEM_TEST_CMD,
        LR1121_MODEM_TEST_MODE_TST_RADIO_RST,
    };

    return ( lr1121_modem_response_code_t ) lr1121_modem_hal_write(
        context, cbuffer, LR1121_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH, 0, 0 );
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

void lr1121_parse_charge_hook_id( const uint8_t* buffer, uint8_t hook_id, lr1121_modem_consumption_details_t* hook )
{
    hook->tx_last_toa_ms      = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 0 );
    hook->rx_last_toa_ms      = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 4 );
    hook->tx_cumulated_toa_ms = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 8 );
    hook->rx_cumulated_toa_ms = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 12 );
    hook->none_consumption_ms = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 16 );
    hook->tx_consumption_ma   = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 20 );
    hook->rx_consumption_ma   = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 24 );
    hook->none_consumption_ma = lr1121_uint8_to_uint32( buffer + 32 * hook_id + 28 );
}

/* --- EOF ------------------------------------------------------------------ */
