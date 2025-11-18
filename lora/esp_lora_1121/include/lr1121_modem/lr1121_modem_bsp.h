/*!
 * @file      lr1121_modem_bsp.h
 *
 * @brief     BSP driver for LR1121 modem
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

#ifndef LR1121_MODEM_BSP_H
#define LR1121_MODEM_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr1121_modem_common.h"
#include "lr1121_modem_bsp_types.h"

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
 * @brief This command gets the board-specific correction offset for transmission power to be used (signed integer in
 * dB).
 *
 * @param [in] context Chip implementation context
 * @param [out] tx_power_offset Tx power offset correction in dBm
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_tx_power_offset( const void* context, int8_t* tx_power_offset );

/*!
 * @brief This command sets the board-specific correction offset for transmission power to be used
 *
 * The offset depends on the board design and antenna matching and is expressed in dB (signed integer).
 * The default value is -2dB.
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_power_offset Tx power offset correction in dBm
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_tx_power_offset( const void* context, const int8_t tx_power_offset );

/*!
 * @brief Configure the Output RF per power
 *
 * This command overload the output RF configuration for the given @p expected_power.
 *
 * @param [in] context Chip implementation context
 * @param [in] output_power_configs Array of Tx output power configurations to set. It is up to the user to ensure it
 * contains at least @p n_output_power_configs
 * @param [in] n_output_power_configs Number of output power configuration to set. Valid values in range [1:41]
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_output_power_config(
    const void* context, const lr1121_modem_output_power_config_t* output_power_configs,
    uint8_t n_output_power_configs );

/*!
 * @brief This command gets the 41 Tx output power configurations for all Tx powers
 *
 * In case an @p expected_power is not available due to hardware limitation, all corresponding values in @p
 * lr1121_modem_output_power_config_t wil be set to 0x7F.
 *
 * @param [in] context Chip implementation context
 * @param [out] output_power_config Tx output power configuration block list, \see
 * lr1121_modem_output_power_config_list_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_output_power_config(
    const void* context, lr1121_modem_output_power_config_list_t output_power_config );

/*!
 * @brief Configure RF output configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] output RF output @ref lr1121_modem_bsp_radio_pa_selection_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_rf_output( const void*                                 context,
                                                         const lr1121_modem_bsp_radio_pa_selection_t output );

/*!
 * @brief Get RF output configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] output RF output @ref lr1121_modem_bsp_radio_pa_selection_t
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_rf_output( const void*                                  context,
                                                         const lr1121_modem_bsp_radio_pa_selection_t* output );

/*!
 * @brief Get the Crystal error
 *
 * @param [in] context Chip implementation context
 * @param [out] crystal_error_ppm Crystal error in PPM
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_crystal_error( const void* context, uint32_t* crystal_error_ppm );

/*!
 * @brief Set the Crystal error of the MCU to fine adjust the rx window for LoRaWAN
 *
 * The default value is:
 *   - When the Low-Frequency clock selected is @ref LR1121_MODEM_SYSTEM_LFCLK_RC : 16000 PPM
 *   - Otherwise: 50 PPM
 *
 * @param [in] context Chip implementation context
 * @param [in] crystal_error_ppm Crystal error in PPM
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_crystal_error( const void* context, const uint32_t crystal_error_ppm );

/*!
 * @brief Get the XOSC trimming capacitor
 *
 * @param [in] context Chip implementation context
 * @param [out] capa_trim_a Trimming value for capacitance XTA
 * @param [out] capa_trim_b Trimming value for capacitance XTB
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_xosc_capa_trim_a_b( const void* context, uint8_t* capa_trim_a,
                                                                  uint8_t* capa_trim_b );

/*!
 * @brief Set the XOSC trimming capacitor
 *
 * @p capa_trim_a and @p capa_trim_b both takes value within [0, 47]. It configures the corresponding trimming
 * capacitance following \f$ C_{x,uF} = 0.47 \times capa\_trim\_x + N_x \f$ Where:
 *   - \f$ N_a = 11.3 \f$
 *   - \f$ N_b = 10.1 \f$
 *
 * So that \f$ C_{a,uF} \f$ goes from 11.3pF to 33.4pF ; and \f$ C_{b,uF} \f$ goes from 10.1pF to 32.2pF.
 *
 * Default value is 0x12 for both @p capa_trim_a and @p capa_trim_b which makes respectively 19.7pF and 18.5pF.
 *
 * Note that when the chip enters sleep mode, the capacitances XTA and XTB are respectively set to 13.6pF and 12.4pF.
 * When leaving sleep mode, XTA and XTB are set to the value configured by this command (or to the default ones).
 *
 * @param [in] context Chip implementation context
 * @param [in] capa_trim_a Trimming value for capacitance XTA
 * @param [in] capa_trim_b Trimming value for capacitance XTB
 *
 * @returns Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_xosc_capa_trim_a_b( const void* context, const uint8_t capa_trim_a,
                                                                  const uint8_t capa_trim_b );

/**
 * @brief Get the instantaneous power consumption table
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [out] consumption_per_power Table of instantaneous consumption
 * @return Operation status
 */
lr1121_modem_response_code_t lr1121_modem_get_tx_power_consumption_ua(
    const void* context, lr1121_modem_tx_power_consumption_list_t consumption_per_power );

/**
 * @brief Set the instantaneous power consumption table
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [in] consumption_per_power Array of the instantaneous power consumption to set. Setting the field
 * consumed_power_ua to 0 reset the instantaneous consumption corresponding to the given Tx RF power to 0. It is up to
 * the caller to ensure this array contains at least n_consumption_per_power elements
 * @param [in] n_consumption_per_power Number of elements in the array @p consumption_per_power to set. Valid values
 * in range of [1:41]
 * @return Operation status
 */
lr1121_modem_response_code_t lr1121_modem_set_tx_power_consumption_ua(
    const void* context, const lr1121_modem_tx_power_consumption_value_t* consumption_per_power,
    uint8_t n_consumption_per_power );

/**
 * @brief Get LoRa Rx consumption configured
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_consumption The LoRa instantaneous Rx consumption configured
 * @return lr1121_modem_response_code_t
 */
lr1121_modem_response_code_t lr1121_modem_get_lora_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption );

/**
 * @brief Set the LoRa Rx power consumption configuration
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_consumption The LoRa Rx consumption structure to set. Setting one field to 0 reset it to its internal
 * value.
 * @return lr1121_modem_response_code_t
 */
lr1121_modem_response_code_t lr1121_modem_set_lora_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption );

/**
 * @brief Get GFSK Rx power consumption configured
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_consumption The GFSK instantaneous Rx consumption configured
 * @return lr1121_modem_response_code_t
 */
lr1121_modem_response_code_t lr1121_modem_get_gfsk_rx_power_consumption_ua(
    const void* context, lr1121_modem_rx_power_consumption_t* rx_consumption );

/**
 * @brief Set the GFSK Rx power consumption configuration
 *
 * The instantaneous power consumption values are used to evaluate the power consumption statistics.
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_consumption The GFSK Rx consumption structure to set. Setting one field to 0 reset it to its internal
 * value.
 * @return lr1121_modem_response_code_t
 */
lr1121_modem_response_code_t lr1121_modem_set_gfsk_rx_power_consumption_ua(
    const void* context, const lr1121_modem_rx_power_consumption_t* rx_consumption );

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_BSP_H

/* --- EOF ------------------------------------------------------------------ */
