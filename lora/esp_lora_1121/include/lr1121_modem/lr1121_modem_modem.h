/*!
 * @file      lr1121_modem_modem.h
 *
 * @brief     Modem driver for LR1121 modem
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

#ifndef LR1121_MODEM_MODEM_H
#define LR1121_MODEM_MODEM_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Reset non-volatile settings to their default value
 *
 * Settings that are reset are:
 * - region
 * - devnonce
 * - joinnonce
 * - LoRaWAN certification state
 * - low frequency clock source
 * - reset counter
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_factory_reset( const void* context );

/*!
 * @brief Return the modem version information
 *
 * @param [in] context Chip implementation context
 * @param [out] modem_version Version of the LR1121 modem
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_modem_version( const void*             context,
                                                             lr1121_modem_version_t* modem_version );

/*!
 * @brief Return the modem status
 *
 * @param [in] context Chip implementation context
 * @param [out] status LR1121 mode status bit mask @ref lr1121_modem_lorawan_status_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_status( const void*                            context,
                                                      lr1121_modem_lorawan_status_bitmask_t* status );

/*!
 * @brief Return and reset the consumption statistics of the modem
 *
 * @param [in] context Chip implementation context
 * @param [out] charge Charge counter in mAh
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_charge( const void* context, lr1121_modem_charge_t* charge );

/*!
 * @brief Get pending events from the modem
 *
 * Pending events are indicated by the EVENT line. The EVENT line will be de-asserted after all events have been
 * retrieved and no further events are available.
 * When no event is available this command returns with empty response payload.
 *
 * @param [in] context Chip implementation context
 * @param [out] event_fields \see lr1121_modem_event_fields_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_event( const void* context, lr1121_modem_event_fields_t* event_fields );

/*!
 * @brief Suspend or resume the modem’s radio operations
 *
 * It can be used to prevent extra power consumption by the modem in case the application MCU temporarily needs more
 * power itself and wants to prevent exceeding limits.
 *
 * @param [in] context Chip implementation context
 * @param [in] suspend Operations are suspended with parameter value 0x01 and resumed with parameter value 0x00 @ref
 * lr1121_modem_suspend_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_suspend( const void* context, const lr1121_modem_suspend_t suspend );

/*!
 * @brief Get suspend or resume state of the the modem’s radio operations.
 *
 * @param [in] context Chip implementation context
 * @param [out] suspend Operations are suspended with parameter value 0x01 and resumed with parameter value 0x00 @ref
 * lr1121_modem_suspend_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_suspend( const void* context, lr1121_modem_suspend_t* suspend );

/*!
 * @brief Set an application alarm timer in seconds
 *
 * When the timer has expired the event @ref LR1121_MODEM_LORAWAN_EVENT_ALARM is generated.
 * If this command is applied again before the timer has expired, the timer will be started again with the new period.
 * Value 0 immediately generates event @ref LR1121_MODEM_LORAWAN_EVENT_ALARM.
 *
 * @param [in] context Chip implementation context
 * @param [in] seconds Seconds
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_alarm_timer( const void* context, uint32_t seconds );

/*!
 * @brief Clear the alarm timer
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_clear_alarm_timer( const void* context );

/*!
 * @brief Get the application alarm timer remaining time
 *
 * @param [in] context Chip implementation context
 * @param [out] remaining_time Remaining time in seconds
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_alarm_remaining_time( const void* context, uint32_t* remaining_time );

/*!
 * @brief Get the crashlog status
 *
 * @param [in] context Chip implementation context
 * @param [out] status Crashlog status, see \ref lr1121_modem_crashlog_status_t
 * @param [out] crashlog Function name in ASCII. It is up to the caller to ensure this pointer is at least 242 bytes
 * long
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_crashlog( const void* context, lr1121_modem_crashlog_status_t* status,
                                                        uint8_t* crashlog );

/*!
 * @brief Enable test functions
 *
 * With the exception of the @ref lr1121_modem_test_mode_start command, test commands are only available if test mode is
 * active. Test mode can only be activated if the modem has not yet received a command that results in a radio
 * operation. Once test mode is active, all other modem commands are disabled.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_mode_start( const void* context );

/*!
 * @brief No operation. This command may be used to terminate an ongoing continuous TX operation.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_nop( const void* context );

/*!
 * @brief Transmit LoRa packets
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] tx_power Tx power in dBm
 * @param [in] payload_length Payload length
 * @param [in] sf spreading factor @ref lr1121_modem_tst_mode_lora_sf_t
 * @param [in] bw bandwidth @ref lr1121_modem_tst_mode_lora_bw_t
 * @param [in] cr Coding Rate @ref lr1121_modem_tst_mode_lora_cr_t
 * @param [in] is_iq_inverted Enable IQ inverted
 * @param [in] is_crc_enabled Enable CRC field
 * @param [in] header_mode Configure header implicit or explicit
 * @param [in] preamble_length Number of preamble symbols
 * @param [in] number_of_tx Number of packet to send. 0 makes the chip to send packet infinitely until call to @ref
 * lr1121_modem_test_nop
 * @param [in] delay_ms Delay between two consecutive transmit
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_tx_lora(
    const void* context, uint32_t frequency, int8_t tx_power, uint8_t payload_length,
    lr1121_modem_tst_mode_lora_sf_t sf, lr1121_modem_tst_mode_lora_bw_t bw, lr1121_modem_tst_mode_lora_cr_t cr,
    bool is_iq_inverted, bool is_crc_enabled, lr1121_modem_tst_mode_lora_packet_header_mode_t header_mode,
    uint32_t preamble_length, uint32_t number_of_tx, uint32_t delay_ms );

/*!
 * @brief Transmit FSK packets
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] tx_power Tx power in dBm
 * @param [in] payload_length Payload length
 * @param [in] number_of_tx Number of packet to send. 0 makes the chip to send packet infinitely until call to @ref
 * lr1121_modem_test_nop
 * @param [in] delay_ms Delay between two consecutive transmit
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_tx_fsk( const void* context, uint32_t frequency, int8_t tx_power,
                                                       uint8_t payload_length, uint32_t number_of_tx,
                                                       uint32_t delay_ms );

/*!
 * @brief Transmit LR-FHSS packets
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] tx_power Tx power in dBm
 * @param [in] payload_length Payload length
 * @param [in] grid LR-FHSS grid
 * @param [in] bw LR-FHSS bandwidth
 * @param [in] cr LR-FHSS coding rate
 * @param [in] number_of_tx Number of packet to send. 0 makes the chip to send packet infinitely until call to @ref
 * lr1121_modem_test_nop
 * @param [in] delay_ms Delay between two consecutive transmit
 * @param [in] is_hopping_enabled Enable LR-FHSS frequency hopping
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_tx_lr_fhss( const void* context, uint32_t frequency, int8_t tx_power,
                                                           uint8_t                              payload_length,
                                                           lr1121_modem_tst_mode_lr_fhss_grid_t grid,
                                                           lr1121_modem_tst_mode_lr_fhss_bw_t   bw,
                                                           lr1121_modem_tst_mode_lr_fhss_cr_t cr, uint32_t number_of_tx,
                                                           uint32_t delay_ms, bool is_hopping_enabled );

/*!
 * @brief Transmit a continuous wave.
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] tx_power Tx power in dBm
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_tx_cw( const void* context, uint32_t frequency, int8_t tx_power );

/*!
 * @brief Continuously receive LoRa packets
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] sf spreading factor @ref lr1121_modem_tst_mode_lora_sf_t
 * @param [in] bw bandwidth @ref lr1121_modem_tst_mode_lora_bw_t
 * @param [in] cr Coding Rate @ref lr1121_modem_tst_mode_lora_cr_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_rx_lora_cont( const void* context, uint32_t frequency,
                                                             lr1121_modem_tst_mode_lora_sf_t sf,
                                                             lr1121_modem_tst_mode_lora_bw_t bw,
                                                             lr1121_modem_tst_mode_lora_cr_t cr );

/*!
 * @brief Continuously receive FSK packets
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_rx_fsk_cont( const void* context, uint32_t frequency );

/*!
 * @brief Read the packet counter received during continuously receive packets test.
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_packet_counter The counter of packet received during RX continuous packet test
 *
 * @returns Operation status
 *
 * @see lr1121_modem_test_rx_lora_cont
 */
lr1121_modem_response_code_t lr1121_modem_test_read_packet_counter_rx_cont( const void* context,
                                                                            uint32_t*   rx_packet_counter );

/*!
 * @brief Continuously receive packets on Sub-GHz radio path.
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency Frequency in Hz
 * @param [in] time_ms time in millisecond of the radio acquisition
 * @param [in] bw_hz bandwidth in Hz
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_rssi_subghz( const void* context, uint32_t frequency, uint16_t time_ms,
                                                            uint32_t bw_hz );

/*!
 * @brief Reset the LR1121 radio.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_radio_rst( const void* context );

/*!
 * @brief Exit test mode and reset LR1121 modem.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_exit( const void* context );

/*!
 * @brief Read RSSI after a Sub Gig / 2.4 Ghz test rssi command.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi RSSI in dBm
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_test_read_rssi( const void* context, int16_t* rssi );

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_MODEM_H

/* --- EOF ------------------------------------------------------------------ */
