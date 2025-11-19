/*!
 * @file      lr1121_modem_bsp_types.h
 *
 * @brief     BSP driver types for LR1121 modem
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

#ifndef LR1121_MODEM_BSP_TYPES_H
#define LR1121_MODEM_BSP_TYPES_H

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

/*!
 * @brief Number of output power config blocks
 */
#define LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS ( 41 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Power Amplifier Selection values
 *
 * - Low-power Power Amplifier can reach up to 14dBm
 * - High-power Power Amplifier can reach up to 22 dBm
 */
typedef enum
{
    LR1121_MODEM_BSP_RADIO_PA_SEL_LP = 0x00,  //!< Low-power Power Amplifier
    LR1121_MODEM_BSP_RADIO_PA_SEL_HP = 0x01,  //!< High-power Power Amplifier
    LR1121_MODEM_BSP_RADIO_PA_SEL_LP_HP_LF =
        0x02,  //!< Automatic selection between Low-power and High-power depending on requested power
    LR1121_MODEM_BSP_RADIO_PA_SEL_HF = 0x03,  //!< High Frequency Power Amplifier
} lr1121_modem_bsp_radio_pa_selection_t;

/*!
 * @brief Ramping time for PA
 *
 * This parameter is the ramping time of the PA. A high value improves spectral quality.
 */
typedef enum
{
    LR1121_MODEM_RAMP_16_US  = 0x00,  //!< 16 us Ramp Time
    LR1121_MODEM_RAMP_32_US  = 0x01,  //!< 32 us Ramp Time
    LR1121_MODEM_RAMP_48_US  = 0x02,  //!< 48 us Ramp Time (Default)
    LR1121_MODEM_RAMP_64_US  = 0x03,  //!< 64 us Ramp Time
    LR1121_MODEM_RAMP_80_US  = 0x04,  //!< 80 us Ramp Time
    LR1121_MODEM_RAMP_96_US  = 0x05,  //!< 96 us Ramp Time
    LR1121_MODEM_RAMP_112_US = 0x06,  //!< 112 us Ramp Time
    LR1121_MODEM_RAMP_128_US = 0x07,  //!< 128 us Ramp Time
    LR1121_MODEM_RAMP_144_US = 0x08,  //!< 144 us Ramp Time
    LR1121_MODEM_RAMP_160_US = 0x09,  //!< 160 us Ramp Time
    LR1121_MODEM_RAMP_176_US = 0x0A,  //!< 176 us Ramp Time
    LR1121_MODEM_RAMP_192_US = 0x0B,  //!< 192 us Ramp Time
    LR1121_MODEM_RAMP_208_US = 0x0C,  //!< 208 us Ramp Time
    LR1121_MODEM_RAMP_240_US = 0x0D,  //!< 240 us Ramp Time
    LR1121_MODEM_RAMP_272_US = 0x0E,  //!< 272 us Ramp Time
    LR1121_MODEM_RAMP_304_US = 0x0F,  //!< 304 us Ramp Time
} lr1121_modem_ramp_time_t;

/*!
 * @brief Select power amplifier supply source
 */
typedef enum
{
    LR1121_MODEM_PA_REG_SUPPLY_VREG = 0x00,  //!< Power amplifier supplied by the main regulator
    LR1121_MODEM_PA_REG_SUPPLY_VBAT = 0x01   //!< Power amplifier supplied by the battery
} lr1121_modem_pa_reg_supply_t;

/*!
 * @brief Power Amplifier selection for RF output configuration table
 */
typedef enum
{
    LR1121_MODEM_OUTPUT_POWER_CONFIGURATION_PA_SEL_LP = 0x00,  //!< Low-power Power Amplifier
    LR1121_MODEM_OUTPUT_POWER_CONFIGURATION_PA_SEL_HP = 0x01,  //!< High-power Power Amplifier
    LR1121_MODEM_OUTPUT_POWER_CONFIGURATION_PA_SEL_HF = 0x02,  //!< High Frequency Power Amplifier
} lr1121_modem_output_power_configuration_pa_sel_t;

/*!
 * @brief Output Power Config structure
 *
 * A power configuration for an @p expected_power can be reset to its internal configuration by setting one of the other
 * structure field to value 0x7F.
 *
 * @p pa_duty_cycle controls the duty cycle of Power Amplifier according to:
 * \f$ dutycycle = 0.2 + 0.04 \times pa\_duty\_cycle \f$
 * It can be used to adapt the TX multi-band operation using a single-matching network.
 *
 * The allowed duty cycle values for LPA are from 0.2 to 0.48 (by step of 0.04). Therefore possible values for
 * pa_duty_cycle go from 0 to 7.
 *
 * The allowed duty cycle values for HPA go from 0.2 to 0.36 (by step of 0.04). Therefore in this case, the possible
 * values for pa_duty_cycle go from 0 to 4.
 *
 * @p pa_hp_sel controls the number of slices for HPA according to: \f$ \#slices = pa\_hp\_sel + 1 \f$
 */
typedef struct
{
    uint8_t                                          expected_power;    //!< Expected power in dBm
    uint8_t                                          configured_power;  //!< Configured power in dBm
    lr1121_modem_pa_reg_supply_t                     pa_supply;         //!< Power Amplifier regulator supply source
    uint8_t                                          pa_duty_cycle;     //!< Power Amplifier duty cycle (Default 0x04)
    lr1121_modem_output_power_configuration_pa_sel_t pa_sel;            //!< Power Amplifier selection
    uint8_t                                          pa_hp_sel;         //!< Number of slices for HPA (Default 0x07)
    lr1121_modem_ramp_time_t                         pa_ramp_time;      //!< Power amplifier ramp time
} lr1121_modem_output_power_config_t;

/*!
 * @brief Output power config type
 */
typedef lr1121_modem_output_power_config_t
    lr1121_modem_output_power_config_list_t[LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS];

/**
 * @brief Tx power consumption mapping structure
 */
typedef struct
{
    int8_t   tx_power_dbm;       //!< Tx RF output power
    uint32_t consumed_power_ua;  //!< Corresponding instantaneous power consumption (uA)
} lr1121_modem_tx_power_consumption_value_t;

/**
 * @brief Power consumption table
 */
typedef lr1121_modem_tx_power_consumption_value_t
    lr1121_modem_tx_power_consumption_list_t[LR1121_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS];

/**
 * @brief Rx power consumption structure
 *
 * The Rx power consumption depends on the Rx boosted configuration.
 *
 * @see lr1121_modem_radio_cfg_rx_boosted
 */
typedef struct
{
    uint32_t consumption_rx_boosted_off_ua;  //!< Instantaneous consumption without Rx boosted disabled (uA)
    uint32_t consumption_rx_boosted_on_ua;   //!< Instantaneous consumption without Rx boosted enable (uA)
} lr1121_modem_rx_power_consumption_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_BSP_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
