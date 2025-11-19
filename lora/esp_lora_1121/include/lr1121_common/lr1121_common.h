#ifndef LR1121_COMMON_H
#define LR1121_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lr11xx_driver/lr11xx_radio_types.h"
#include "lr11xx_driver/lr11xx_radio.h"

#include "lr11xx_driver/lr11xx_system_types.h"
#include "lr11xx_driver/lr11xx_regmem.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

 #define SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_MIN 150000000
 #define SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_MAX 960000000
 
 #define SMTC_SHIELD_LR112X_2GHZ_FREQ_MIN 2000000000
 #define SMTC_SHIELD_LR112X_2GHZ_FREQ_MAX 2100000000
 
 #define SMTC_SHIELD_LR112X_2_4GHZ_FREQ_MIN 2400000000
 #define SMTC_SHIELD_LR112X_2_4GHZ_FREQ_MAX 2500000000
 
 #define SMTC_SHIELD_LR11XX_MIN_PWR -9
 #define SMTC_SHIELD_LR11XX_MAX_PWR 22
 
 #define SMTC_SHIELD_LR112X_MIN_PWR_HF -18
 #define SMTC_SHIELD_LR112X_MAX_PWR_HF 13

 /*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

 /*!
 * @brief Helper macro that returned a human-friendly message if a command does not return LR11XX_STATUS_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref lr11xx_status_t
 *
 * @param[in] rc  Return code
 */
#define ASSERT_LR11XX_RC( rc )                                                                   \
{                                                                                            \
    const lr11xx_status_t status = rc;                                                       \
    if( status != LR11XX_STATUS_OK )                                                         \
    {                                                                                        \
        if( status == LR11XX_STATUS_ERROR )                                                  \
        {                                                                                    \
            printf( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, \
                                 xstr( LR11XX_STATUS_ERROR ) );                              \
        }                                                                                    \
    }                                                                                        \
}

 /*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

 /*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Power amplifier and output power configurations structure definition
 */
typedef struct smtc_shield_lr11xx_pa_pwr_cfg_s
{
    int8_t                power;
    lr11xx_radio_pa_cfg_t pa_config;
} smtc_shield_lr11xx_pa_pwr_cfg_t;

/**
 * @brief External 32MHz oscillator configuration structure definition
 */
typedef struct smtc_shield_lr11xx_xosc_cfg_s
{
    bool                                has_tcxo;
    lr11xx_system_tcxo_supply_voltage_t supply;
    uint32_t                            startup_time_in_tick;
} smtc_shield_lr11xx_xosc_cfg_t;

/**
 * @brief 32kHz clock configuration structure definition
 */
typedef struct smtc_shield_lr11xx_lfclk_cfg_s
{
    lr11xx_system_lfclk_cfg_t lf_clk_cfg;
    bool                      wait_32k_ready;
} smtc_shield_lr11xx_lfclk_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @see smtc_shield_lr11xx_get_rssi_calibration_table
 */
const lr11xx_radio_rssi_calibration_table_t* smtc_shield_lr11xx_get_rssi_calibration_table(
    const uint32_t rf_freq_in_hz );

 /**
 * @see smtc_shield_lr11xx_get_rf_switch_cfg
 */
const lr11xx_system_rfswitch_cfg_t* smtc_shield_lr11xx_common_get_rf_switch_cfg( void );

/**
 * @see smtc_shield_lr11xx_get_reg_mode
 */
lr11xx_system_reg_mode_t smtc_shield_lr11xx_common_get_reg_mode( void );

/**
 * @see smtc_shield_lr11xx_get_lfclk_cfg
 */
const smtc_shield_lr11xx_lfclk_cfg_t* smtc_shield_lr11xx_common_get_lfclk_cfg( void );

/**
 * @see smtc_shield_lr11xx_get_pa_pwr_cfg
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr1121mb1gis_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
    int8_t expected_output_pwr_in_dbm );

const uint8_t smtc_shield_lr11xx_common_compute_lora_ldro( const lr11xx_radio_lora_sf_t sf, const lr11xx_radio_lora_bw_t bw );

/*!
 * \brief Given the length of a BPSK frame, in bits, calculate the space necessary to hold the frame after differential
 * encoding, in bits.
 *
 * \param [in]  bpsk_pld_len_in_bits Length of a BPSK frame, in bits
 * \returns                          Space required for DBPSK frame, after addition of start/stop bits [bits]
 */
static inline int smtc_dbpsk_get_pld_len_in_bits( int bpsk_pld_len_in_bits )
{
    // Hold the last bit one extra bit-time
    return bpsk_pld_len_in_bits + 2;
}

/*!
 * \brief Given the length of a BPSK frame, in bits, calculate the space necessary to hold the frame after differential
 * encoding, in bytes.
 *
 * \param [in]  bpsk_pld_len_in_bits Length of a BPSK frame, in bits
 * \returns                          Space required for DBPSK frame, after addition of start/stop bits [bytes]
 */
static inline int smtc_dbpsk_get_pld_len_in_bytes( int bpsk_pld_len_in_bits )
{
    return ( smtc_dbpsk_get_pld_len_in_bits( bpsk_pld_len_in_bits ) + 7 ) >> 3;
}


#ifdef __cplusplus
}
#endif

#endif