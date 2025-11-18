/*!
 * @file      lr1121_modem_helper.c
 *
 * @brief     helper functions implementation for LR1121 modem
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

#include "lr1121_modem_helper.h"
#include "lr1121_modem_common.h"
#include "lr1121_modem_modem.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1121_modem_helper_status_t lr1121_modem_helper_get_event_data( const void*           context,
                                                                 lr1121_modem_event_t* modem_event )
{
    lr1121_modem_helper_status_t status       = LR1121_MODEM_HELPER_STATUS_ERROR;
    lr1121_modem_event_fields_t  event_fields = { 0 };

    const lr1121_modem_response_code_t modem_response_code = lr1121_modem_get_event( context, &event_fields );

    if( modem_response_code == LR1121_MODEM_RESPONSE_CODE_OK )
    {
        status                     = LR1121_MODEM_HELPER_STATUS_OK;
        modem_event->event_type    = event_fields.event_type;
        modem_event->missed_events = event_fields.missed_events_count;

        switch( modem_event->event_type )
        {
        case LR1121_MODEM_LORAWAN_EVENT_RESET:
            modem_event->event_data.reset.count = event_fields.data;
            break;
        case LR1121_MODEM_LORAWAN_EVENT_TX_DONE:
            modem_event->event_data.txdone.status = ( lr1121_modem_tx_done_event_t )( event_fields.data >> 8 );
            break;
        case LR1121_MODEM_LORAWAN_EVENT_LINK_CHECK:
            modem_event->event_data.link_check.status = ( lr1121_modem_link_check_event_t )( event_fields.data >> 8 );
            break;
        case LR1121_MODEM_LORAWAN_EVENT_LORAWAN_MAC_TIME:
            modem_event->event_data.mac_time.status = ( lr1121_modem_mac_time_event_t )( event_fields.data >> 8 );
            break;
        case LR1121_MODEM_LORAWAN_EVENT_CLASS_B_PING_SLOT_INFO:
            modem_event->event_data.ping_slot_info.status =
                ( lr1121_modem_class_b_ping_slot_info_t )( event_fields.data >> 8 );
            break;
        case LR1121_MODEM_LORAWAN_EVENT_CLASS_B_STATUS:
            modem_event->event_data.ping_slot_status.status =
                ( lr1121_modem_class_b_ping_slot_status_t )( event_fields.data >> 8 );
            break;
        case LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
        {
            modem_event->event_data.new_multicast_class_c_groupid.mc_group_id = ( uint8_t )( event_fields.data >> 8 );
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
        {
            modem_event->event_data.new_multicast_class_b_groupid.mc_group_id = ( uint8_t )( event_fields.data >> 8 );
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_DYNAMIC:
        {
            modem_event->event_data.relay_tx_dynamic_status.status =
                ( lr1121_modem_relay_tx_dynamic_status_t )( event_fields.data >> 8 );
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_MODE:
        {
            modem_event->event_data.relay_tx_mode_status.status =
                ( lr1121_modem_relay_tx_mode_status_t )( event_fields.data >> 8 );
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_SYNC:
        {
            modem_event->event_data.relay_tx_sync_status.status =
                ( lr1121_modem_relay_tx_sync_status_t )( event_fields.data >> 8 );
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_FUOTA_DONE:
        {
            modem_event->event_data.fuota_status.status =
                ( lr1121_modem_fuota_status_t )( event_fields.data >> 8 ) & 0x00FF;
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_TEST_MODE:
        {
            modem_event->event_data.test_mode_status.status =
                ( lr1121_modem_test_mode_status_t )( event_fields.data >> 8 ) & 0x00FF;
            break;
        }
        case LR1121_MODEM_LORAWAN_EVENT_REGIONAL_DUTY_CYCLE:
        {
            modem_event->event_data.regional_duty_cycle_status.status =
                ( lr1121_modem_regional_duty_cycle_status_t )( ( uint8_t )( event_fields.data >> 8 ) );
            break;
        }
        default:
            break;
        }
    }

    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
