/*!
 * @file      lr1121_modem_helper.h
 *
 * @brief     helper functions definition for LR1121 modem
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

#ifndef LR1121_MODEM_HELPER_H
#define LR1121_MODEM_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr1121_modem_modem_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LR1121 Modem helper status
 */
typedef enum
{
    LR1121_MODEM_HELPER_STATUS_OK    = 0,
    LR1121_MODEM_HELPER_STATUS_ERROR = 3,
} lr1121_modem_helper_status_t;

/*!
 * @brief TX status values
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_TX_DONE
 */
typedef enum
{
    LR1121_MODEM_TX_NOT_SENT    = 0x00,
    LR1121_MODEM_UNCONFIRMED_TX = 0x01,
    LR1121_MODEM_CONFIRMED_TX   = 0x02,
} lr1121_modem_tx_done_event_t;

/*!
 * @brief Link check request values
 */
typedef enum
{
    LR1121_MODEM_LINK_CHECK_NOT_RECEIVED = 0x00,  //!< Link check response has not been received
    LR1121_MODEM_LINK_CHECK_RECEIVED     = 0x01,  //!< Link check response has been received
} lr1121_modem_link_check_event_t;

/*!
 * @brief Time request values
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_LORAWAN_MAC_TIME
 */
typedef enum
{
    LR1121_MODEM_TIME_NOT_VALID          = 0x00,  //!< Time is not valid
    LR1121_MODEM_TIME_VALID              = 0x01,  //!< Time is valid and has been synchronized
    LR1121_MODEM_TIME_VALID_BUT_NOT_SYNC = 0x02,  //!< Time is still valid but has not been synchronized
} lr1121_modem_mac_time_event_t;

/*!
 * @brief class b ping slot info
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_CLASS_B_PING_SLOT_INFO
 */
typedef enum
{
    LR1121_MODEM_CLASS_B_PING_SLOT_INFO_NOT_ANSWERED = 0x00,
    LR1121_MODEM_CLASS_B_PING_SLOT_INFO_ANSWERED     = 0x01,
} lr1121_modem_class_b_ping_slot_info_t;

/*!
 * @brief class b ping slot status
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_CLASS_B_STATUS
 */
typedef enum
{
    LR1121_MODEM_CLASS_B_PING_SLOT_STATUS_NOT_READY = 0x00,
    LR1121_MODEM_CLASS_B_PING_SLOT_STATUS_READY     = 0x01,
} lr1121_modem_class_b_ping_slot_status_t;

/*!
 * @brief Event status for Wake On Radio protocol status change
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_DYNAMIC
 */
typedef enum
{
    LR1121_MODEM_RELAY_TX_DYNAMIC_WOR_DISABLED = 0x00,
    LR1121_MODEM_RELAY_TX_DYNAMIC_WOR_ENABLED  = 0x01,
} lr1121_modem_relay_tx_dynamic_status_t;

/*!
 * @brief Event status for relay Tx activation change
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_MODE
 */
typedef enum
{
    LR1121_MODEM_RELAY_TX_MODE_DISABLED          = 0x00,
    LR1121_MODEM_RELAY_TX_MODE_ENABLED           = 0x01,
    LR1121_MODEM_RELAY_TX_MODE_DYNAMIC           = 0x02,
    LR1121_MODEM_RELAY_TX_MODE_DEVICE_CONTROLLED = 0x03,
} lr1121_modem_relay_tx_mode_status_t;

/*!
 * @brief Event status for relay synchronization change
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_SYNC
 */
typedef enum
{
    LR1121_MODEM_RELAY_TX_SYNCHRONIZATION_INIT = 0x00,
    LR1121_MODEM_RELAY_TX_UNSYNCHRONIZED       = 0x01,
    LR1121_MODEM_RELAY_TX_SYNCHRONIZED         = 0x02,
} lr1121_modem_relay_tx_sync_status_t;

/**
 * @brief Event status on FUOTA done event
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_FUOTA_DONE
 */
typedef enum
{
    LR1121_MODEM_FUOTA_STATUS_TERMINATED_SUCCESSFULLY = 0x00,  //!< FUOTA terminated successfully
    LR1121_MODEM_FUOTA_STATUS_FAILED                  = 0x01,  //!< FUOTA failed
    LR1121_MODEM_FUOTA_STATUS_ONGOING                 = 0xFF,  //!< FUOTA is ongoing
    LR1121_MODEM_FUOTA_STATUS_NOT_STARTED             = 0xFE,  //!< FUOTA is not started
} lr1121_modem_fuota_status_t;

/**
 * @brief Event status on test mode event
 *
 * @see LR1121_MODEM_LORAWAN_EVENT_TEST_MODE
 */
typedef enum
{
    LR1121_MODEM_TEST_MODE_STATUS_TX_NOT_SENT       = 0x00,
    LR1121_MODEM_TEST_MODE_STATUS_TX_SENT           = 0x01,
    LR1121_MODEM_TEST_MODE_STATUS_TERMINATED_ACTION = 0x02,
} lr1121_modem_test_mode_status_t;

/**
 * @brief Event status on regional duty cycle event
 *
 * The duty cycle status can be obtained by calling @ref lr1121_modem_get_duty_cycle_status.
 *
 * @see lr1121_modem_get_duty_cycle_status
 */
typedef enum
{
    LR1121_MODEM_REGINAL_DUTY_CYCLE_TX_ALLOWED =
        0x00,  //!< Previously duty cycle constrained transmissions are now allowed
    LR1121_MODEM_REGINAL_DUTY_CYCLE_TX_CONSTRAINED = 0x01,  //!< Transmissions are constrained by regional duty cycle
} lr1121_modem_regional_duty_cycle_status_t;

/**
 * @brief Structure holding event-related data
 */
typedef struct
{
    lr1121_modem_lorawan_event_type_t event_type;     //!< Type of the event
    uint8_t                           missed_events;  //!< Number of @p event_type events missed before the current one
    union
    {
        struct
        {
            uint16_t count;
        } reset;
        struct
        {
            lr1121_modem_tx_done_event_t status;
        } txdone;
        struct
        {
            lr1121_modem_link_check_event_t status;
        } link_check;
        struct
        {
            lr1121_modem_mac_time_event_t status;
        } mac_time;
        struct
        {
            lr1121_modem_class_b_ping_slot_info_t status;
        } ping_slot_info;
        struct
        {
            lr1121_modem_class_b_ping_slot_status_t status;
        } ping_slot_status;
        struct
        {
            uint8_t mc_group_id;
        } new_multicast_class_c_groupid;
        struct
        {
            uint8_t mc_group_id;
        } new_multicast_class_b_groupid;
        struct
        {
            lr1121_modem_relay_tx_dynamic_status_t status;
        } relay_tx_dynamic_status;
        struct
        {
            lr1121_modem_relay_tx_mode_status_t status;
        } relay_tx_mode_status;
        struct
        {
            lr1121_modem_relay_tx_sync_status_t status;
        } relay_tx_sync_status;
        struct
        {
            lr1121_modem_fuota_status_t status;
        } fuota_status;
        struct
        {
            lr1121_modem_test_mode_status_t status;
        } test_mode_status;
        struct
        {
            lr1121_modem_regional_duty_cycle_status_t status;
        } regional_duty_cycle_status;
    } event_data;  //!< Status data associated to the event
} lr1121_modem_event_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Extract the event data contained in the event field buffer
 *
 * @param [in] context Chip implementation context
 * @param [out] modem_event Struct containing the event data \see lr1121_modem_event_t
 *
 * @returns  Operation status
 */
lr1121_modem_helper_status_t lr1121_modem_helper_get_event_data( const void*           context,
                                                                 lr1121_modem_event_t* modem_event );

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_HELPER_H

/* --- EOF ------------------------------------------------------------------ */
