/*!
 * @file      lr1121_modem_relay_types.h
 *
 * @brief     Relay driver types for LR1121 modem
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

#ifndef LR1121_MODEM_RELAY_TYPES_H
#define LR1121_MODEM_RELAY_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

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
 * @brief Relay activation
 *
 * Refer to LoRaWAN Relay specification document for details.
 */
typedef enum
{
    LR1121_MODEM_RELAY_ACTIVATION_DISABLE = 0x00,  //!< The relay is disabled
    LR1121_MODEM_RELAY_ACTIVATION_ENABLE =
        0x01,  //!< The relay is enabled, even if no WOR ack nor downlink on RxR is received
    LR1121_MODEM_RELAY_ACTIVATION_DYNAMIC =
        0x02,  //!< The device automatically enable the relay if no downlink is received after several uplinks
    LR1121_MODEM_RELAY_ACTIVATION_DEVICE_CONTROLLED = 0x03,  //!< The device automatically enable or disable the relay
} lr1121_modem_relay_activation_t;

/**
 * @brief Smart level configuration
 *
 * Refer to LoRaWAN Relay specification document for details.
 */
typedef enum
{
    LR1121_MODEM_RELAY_SMART_LEVEL_8 =
        0x00,  //!< The relay shall be enabled if no valid downlink is received during 8 consecutive uplinks
    LR1121_MODEM_RELAY_SMART_LEVEL_16 =
        0x01,  //!< The relay shall be enabled if no valid downlink is received during 16 consecutive uplinks
    LR1121_MODEM_RELAY_SMART_LEVEL_32 =
        0x02,  //!< The relay shall be enabled if no valid downlink is received during 32 consecutive uplinks
    LR1121_MODEM_RELAY_SMART_LEVEL_64 =
        0x03,  //!< The relay shall be enabled if no valid downlink is received during 64 consecutive uplinks
} lr1121_modem_relay_smart_level_t;

/**
 * @brief Configuration of relay Tx
 */
typedef struct
{
    uint32_t wor_second_channel_frequency_hz;  //!< Frequency of the second Wake On Radio channel (Hz)
    uint32_t
            wor_ack_second_channel_frequency_hz;  //!< Frequency of the second Wake On Radio acknowledgment channel (Hz)
    uint8_t wor_second_channel_datarate;          //!< Datarate of the second Wake On Radio chanel
    uint8_t wor_second_channel_enable;            //!< Wake On Radio second channel enable
    uint8_t backoff_behavior;  //!< Indicate number of missed ACK tolerated before sending the LoRaWAN uplink. Possible
                               //!< values in [0:63] (0 means the LoRaWAN uplink is always sent)
    lr1121_modem_relay_activation_t  activation;   //!< Relay activation configuration
    lr1121_modem_relay_smart_level_t smart_level;  //!< Smart level configuration. Only valid if @ref activation is set
                                                   //!< to @ref LR1121_MODEM_RELAY_ACTIVATION_DYNAMIC
    uint8_t missed_ack_to_unsynchronized_threshold;  //!< Number of consecutively WOR ACK to miss before switching to
                                                     //!< unsynchronized state
} lr1121_modem_relay_tx_configuration_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_RELAY_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
