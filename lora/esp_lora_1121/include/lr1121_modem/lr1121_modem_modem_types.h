/*!
 * @file      lr1121_modem_modem_types.h
 *
 * @brief     Modem driver types for LR1121 modem
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

#ifndef LR1121_MODEM_MODEM_TYPES_H
#define LR1121_MODEM_MODEM_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1121_modem_common.h"

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

/*!
 * @brief Modem status bits
 */
typedef enum
{
    LR1121_LORAWAN_CRASH   = 0x02,
    LR1121_LORAWAN_JOINED  = 0x08,
    LR1121_LORAWAN_SUSPEND = 0x10,
    LR1121_LORAWAN_JOINING = 0x40,
} lr1121_modem_lorawan_status_t;

/**
 * @brief Bit mask for lr1121_modem_lorawan_status_t
 *
 * @see lr1121_modem_lorawan_status_t
 */
typedef uint8_t lr1121_modem_lorawan_status_bitmask_t;

/*!
 * @brief LoRa spreading factor for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LORA_SF5  = 0x05,
    LR1121_MODEM_TST_MODE_LORA_SF6  = 0x06,
    LR1121_MODEM_TST_MODE_LORA_SF7  = 0x07,
    LR1121_MODEM_TST_MODE_LORA_SF8  = 0x08,
    LR1121_MODEM_TST_MODE_LORA_SF9  = 0x09,
    LR1121_MODEM_TST_MODE_LORA_SF10 = 0x0A,
    LR1121_MODEM_TST_MODE_LORA_SF11 = 0x0B,
    LR1121_MODEM_TST_MODE_LORA_SF12 = 0x0C,
} lr1121_modem_tst_mode_lora_sf_t;

/*!
 * @brief LoRa bandwidth for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LORA_10_KHZ  = 0x01,
    LR1121_MODEM_TST_MODE_LORA_15_KHZ  = 0x02,
    LR1121_MODEM_TST_MODE_LORA_20_KHZ  = 0x03,
    LR1121_MODEM_TST_MODE_LORA_31_KHZ  = 0x04,
    LR1121_MODEM_TST_MODE_LORA_41_KHZ  = 0x05,
    LR1121_MODEM_TST_MODE_LORA_62_KHZ  = 0x06,
    LR1121_MODEM_TST_MODE_LORA_125_KHZ = 0x07,
    LR1121_MODEM_TST_MODE_LORA_200_KHZ = 0x08,
    LR1121_MODEM_TST_MODE_LORA_250_KHZ = 0x09,
    LR1121_MODEM_TST_MODE_LORA_400_KHZ = 0x0A,
    LR1121_MODEM_TST_MODE_LORA_500_KHZ = 0x0B,
    LR1121_MODEM_TST_MODE_LORA_800_KHZ = 0x0C,
} lr1121_modem_tst_mode_lora_bw_t;

/*!
 * @brief LoRa coding rate for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LORA_CR_4_5                   = 0x01,
    LR1121_MODEM_TST_MODE_LORA_CR_4_6                   = 0x02,
    LR1121_MODEM_TST_MODE_LORA_CR_4_7                   = 0x03,
    LR1121_MODEM_TST_MODE_LORA_CR_4_8                   = 0x04,
    LR1121_MODEM_TST_MODE_LORA_CR_4_5_LONG_INTERLEAVING = 0x05,
    LR1121_MODEM_TST_MODE_LORA_CR_4_6_LONG_INTERLEAVING = 0x06,
    LR1121_MODEM_TST_MODE_LORA_CR_4_8_LONG_INTERLEAVING = 0x07,
} lr1121_modem_tst_mode_lora_cr_t;

/**
 * @brief LR_FHSS grid for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LR_FHSS_GRID_V1_25391_HZ = 0x00,
    LR1121_MODEM_TST_MODE_LR_FHSS_GRID_V1_3906_HZ  = 0x01,
} lr1121_modem_tst_mode_lr_fhss_grid_t;

/**
 * @brief LR-FHSS bandwidth for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_39063_HZ   = 0x00,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_85938_HZ   = 0x01,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_136719_HZ  = 0x02,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_183594_HZ  = 0x03,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_335938_HZ  = 0x04,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_386719_HZ  = 0x05,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_722656_HZ  = 0x06,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_773438_HZ  = 0x07,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_1523438_HZ = 0x08,
    LR1121_MODEM_TST_MODE_LR_FHSS_BW_V1_1574219_HZ = 0x09,
} lr1121_modem_tst_mode_lr_fhss_bw_t;

/**
 * @brief LR-FHSS coding rate for test mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LR_FHSS_CR_V1_2_3 = 0x01,
    LR1121_MODEM_TST_MODE_LR_FHSS_CR_V1_1_3 = 0x03,
} lr1121_modem_tst_mode_lr_fhss_cr_t;

/*!
 * @brief Packet header mode
 */
typedef enum
{
    LR1121_MODEM_TST_MODE_LORA_PACKET_HEADER_EXPLICIT = 0x00,
    LR1121_MODEM_TST_MODE_LORA_PACKET_HEADER_IMPLICIT = 0x01,
} lr1121_modem_tst_mode_lora_packet_header_mode_t;

/*!
 * @brief Modem suspend type
 */
typedef enum
{
    LR1121_MODEM_RESUMED = 0x00,
    LR1121_MODEM_SUSPEND = 0x01,
} lr1121_modem_suspend_t;

/*!
 * @brief Modem crashlog status type
 */
typedef enum
{
    LR1121_NO_NEW_CRASHLOG = 0x00,
    LR1121_NEW_CRASHLOG    = 0x01,
} lr1121_modem_crashlog_status_t;

/*!
 * @brief LR1121 modem consumption details
 */
typedef struct
{
    uint32_t tx_last_toa_ms;       //!< Last Tx time-on-air (ms)
    uint32_t rx_last_toa_ms;       //!< Last Rx time-on-air (ms)
    uint32_t tx_cumulated_toa_ms;  //!< Cumulated Tx time-on-air (ms)
    uint32_t rx_cumulated_toa_ms;  //!< Cumulated Rx time-on-air (ms)
    uint32_t none_consumption_ms;  //!< Cumulated time not in Tx nor in Rx (ms)
    uint32_t tx_consumption_ma;    //!< Cumulated Tx power consumption (1/1000*uA*ms)
    uint32_t rx_consumption_ma;    //!< Cumulated Rx power consumption (1/1000*uA*ms)
    uint32_t none_consumption_ma;  //!< Cumulated power consumption not in Tx nor in Rx (1/1000*uA*ms)
} lr1121_modem_consumption_details_t;

/*!
 * @brief LR1121 modem consumption per stack action
 */
typedef struct
{
    lr1121_modem_consumption_details_t
        suspend;  //!< Consumptions when modem is in suspend state. These will always read 0,
                  //!< even if transceiver commands where called during this state
    lr1121_modem_consumption_details_t class_b_beacon;  //!< Consumptions of modem related to class B beacon
    lr1121_modem_consumption_details_t lr1mac_stack;    //!< Consumption of modem corresponding to LoRaWAN stack
    lr1121_modem_consumption_details_t lbt;             //!< Consumptions of modem related to Listen Before Talk
    lr1121_modem_consumption_details_t cad;             //!< Consumption of modem related to Channel Activity Detection
    lr1121_modem_consumption_details_t class_b_ping_slot;  //!< Consumptions of modem related to class B ping slots
    lr1121_modem_consumption_details_t test_mode;          //!< Consumptions of modem related to test mode
    lr1121_modem_consumption_details_t
        direct_rp_access;  //!< Consumptions when modem is in direct radio planner access state. These will always read
                           //!< 0, even if transceiver commands where called during this state
    lr1121_modem_consumption_details_t relay_tx;  //!< Consumption of modem related to relay Tx
    lr1121_modem_consumption_details_t class_c;   //!< Consumptions of modem related to Class C
} lr1121_modem_charge_t;

/*!
 * @brief LR1121 modem version structure
 */
typedef struct
{
    uint8_t use_case;     //!< Will always read 5 for LR1121 Modem-E
    uint8_t modem_major;  //!< Major number of Modem-E
    uint8_t modem_minor;  //!< Minor number of Modem-E
    uint8_t modem_patch;  //!< Patch number of Modem-E
    uint8_t lbm_major;    //!< Major number of built-in LoRa Basics Modem
    uint8_t lbm_minor;    //!< Minor number of built-in LoRa Basics Modem
    uint8_t lbm_patch;    //!< Patch number of built-in LoRa Basics Modem
} lr1121_modem_version_t;

/*!
 * @brief Event type for modem operation
 */
typedef enum
{
    LR1121_MODEM_LORAWAN_EVENT_RESET                             = 0x00,  //!< Modem has reset
    LR1121_MODEM_LORAWAN_EVENT_ALARM                             = 0x01,  //!< Alarm time expired
    LR1121_MODEM_LORAWAN_EVENT_JOINED                            = 0x02,  //!< Network successfully joined
    LR1121_MODEM_LORAWAN_EVENT_JOIN_FAIL                         = 0x03,  //!< Attempt to join network failed
    LR1121_MODEM_LORAWAN_EVENT_TX_DONE                           = 0x04,  //!< Frame transmitted
    LR1121_MODEM_LORAWAN_EVENT_DOWN_DATA                         = 0x05,  //!< Downlink data received
    LR1121_MODEM_LORAWAN_EVENT_LINK_CHECK                        = 0x06,  //!< Modem received a LinkADR request
    LR1121_MODEM_LORAWAN_EVENT_LORAWAN_MAC_TIME                  = 0x07,  //!< Modem received a LinkADR request
    LR1121_MODEM_LORAWAN_EVENT_CLASS_B_PING_SLOT_INFO            = 0x08,  //!< Ping Slot Info answered or not by network
    LR1121_MODEM_LORAWAN_EVENT_CLASS_B_STATUS                    = 0x09,  //!< Downlink class B is ready or not
    LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_C     = 0x0A,  //!< New active multicast session in class C
    LR1121_MODEM_LORAWAN_EVENT_NEW_MULTICAST_SESSION_CLASS_B     = 0x0B,  //!< New active multicast session in class B
    LR1121_MODEM_LORAWAN_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C = 0x0C,  //!< End of multicast session in class C
    LR1121_MODEM_LORAWAN_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B = 0x0D,  //!< End of multicast session in class B
    LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_DYNAMIC =
        0x0E,  //!< Relay Tx dynamic mode has enabled or disabled the Wake On Radio
    LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_MODE = 0x0F,  //!< Relay Tx activation has been updated
    LR1121_MODEM_LORAWAN_EVENT_RELAY_TX_SYNC = 0x10,  //!< Relay Tx synchronization has changed
    LR1121_MODEM_LORAWAN_EVENT_ALC_SYNC_TIME = 0x11,  //!< Applicative Layer Clock is synchronized
    LR1121_MODEM_LORAWAN_EVENT_FUOTA_DONE    = 0x12,  //!< FUOTA file download terminated
    LR1121_MODEM_LORAWAN_EVENT_TEST_MODE     = 0x13,  //!< Test mode event
    LR1121_MODEM_LORAWAN_EVENT_REGIONAL_DUTY_CYCLE =
        0x14,  //!< Transmissions are blocked or allowed per regional duty cycle limitation
} lr1121_modem_lorawan_event_type_t;

/*!
 * @brief modem event fields structure
 */
typedef struct
{
    lr1121_modem_lorawan_event_type_t event_type;           //!< Event type
    uint8_t                           missed_events_count;  //!< Counter of missed events of type @p event_type
    uint16_t                          data;                 //!< Event data
} lr1121_modem_event_fields_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_MODEM_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
