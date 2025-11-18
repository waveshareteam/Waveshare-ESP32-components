/*!
 * @file      lr1121_config.c
 *
 * @brief     Common functions shared by the examples
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
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
 *
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
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "lr1121_config.h"

lr1121_t lr1121;

// LoRa modulation parameters
static lr11xx_radio_mod_params_lora_t lora_mod_params = {
  .sf   = LORA_SPREADING_FACTOR,  // Spreading factor
  .bw   = LORA_BANDWIDTH,         // Bandwidth
  .cr   = LORA_CODING_RATE,       // Coding rate
  .ldro = 0  // Low Data Rate Optimization (initialized in radio init)
};

// LoRa packet parameters
static const lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
  .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,  // Preamble length in symbols
  .header_type          = LORA_PKT_LEN_MODE,     // Header type (implicit or explicit)
  .pld_len_in_bytes     = PAYLOAD_LENGTH,        // Payload length in bytes
  .crc                  = LORA_CRC,              // CRC mode
  .iq                   = LORA_IQ,               // IQ inversion
};

// GFSK modulation parameters
static const lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
    .br_in_bps    = FSK_BITRATE,              // Bitrate in bps
    .pulse_shape  = FSK_PULSE_SHAPE,          // Pulse shape
    .bw_dsb_param = FSK_BANDWIDTH,            // Bandwidth parameter
    .fdev_in_hz   = FSK_FDEV,                 // Frequency deviation in Hz
};

// GFSK packet parameters
static const lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
    .preamble_len_in_bits  = FSK_PREAMBLE_LENGTH,  // Preamble length in bits
    .preamble_detector     = FSK_PREAMBLE_DETECTOR, // Preamble detector type
    .sync_word_len_in_bits = FSK_SYNCWORD_LENGTH,   // Sync word length in bits
    .address_filtering     = FSK_ADDRESS_FILTERING, // Address filtering mode
    .header_type           = FSK_HEADER_TYPE,       // Header type
    .pld_len_in_bytes      = PAYLOAD_LENGTH,        // Payload length in bytes
    .crc_type              = FSK_CRC_TYPE,          // CRC type
    .dc_free               = FSK_DC_FREE,           // DC-free encoding mode
};

static const lr11xx_radio_mod_params_bpsk_t bpsk_mod_params = {
    .br_in_bps   = BPSK_BITRATE_IN_BPS,
    .pulse_shape = LR11XX_RADIO_DBPSK_PULSE_SHAPE,
};

static lr11xx_radio_pkt_params_bpsk_t bpsk_pkt_params = {
    .pld_len_in_bytes = 0,  // Will be initialized in radio init
    .ramp_up_delay    = 0,
    .ramp_down_delay  = 0,
    .pld_len_in_bits  = 0,  // Will be initialized in radio init
};

void print_lora_configuration( void );
void print_gfsk_configuration( void );

// Initialize the LR1121 system
void lora_system_init( const void* context )
{
    lr11xx_system_reset( ( void* ) context ); // Reset the LR1121 system
    lr11xx_hal_wakeup( ( void* ) context );   // Wake up the device

    // Enable or disable CRC over SPI
#if defined(USE_LR11XX_CRC_OVER_SPI)
    lr11xx_system_enable_spi_crc(( void* ) context, true);
#else
    lr11xx_system_enable_spi_crc(( void* ) context, false);
#endif    
    
    // Set the LR1121 to standby mode using the external oscillator
    lr11xx_system_set_standby(( void* ) context, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    // Calibrate the image
    lr11xx_system_calibrate_image(( void* ) context,0x6B,0x6E); // Calibrate for 430~440MHz
    // lr11xx_system_calibrate_image(( void* ) context,0xD7,0xDB); // Calibrate for 863~870MHz
    
    // Configure the regulator mode
    const lr11xx_system_reg_mode_t regulator = smtc_shield_lr11xx_common_get_reg_mode();
    lr11xx_system_set_reg_mode( ( void* ) context, regulator );

    // Configure the RF switch
    const lr11xx_system_rfswitch_cfg_t* rf_switch_setup = smtc_shield_lr11xx_common_get_rf_switch_cfg();
    lr11xx_system_set_dio_as_rf_switch( context, rf_switch_setup );

    // Enable the TCXO
    lr11xx_system_set_tcxo_mode( context, LR11XX_SYSTEM_TCXO_CTRL_3_0V, 300 );
    
    // Configure the low-frequency clock source
    lr11xx_system_cfg_lfclk( context, LR11XX_SYSTEM_LFCLK_XTAL, true );

    // Clear all pending error flags
    lr11xx_system_clear_errors( context );
    // Calibrate the system
    lr11xx_system_calibrate( context, 0x3F );

    uint16_t errors;
    // Retrieve system errors
    lr11xx_system_get_errors( context, &errors );
    if(errors & LR11XX_SYSTEM_ERRORS_IMG_CALIB_MASK)
    {
      printf("Image calibration error\r\n");
    }
    // Clear all pending error flags
    lr11xx_system_clear_errors( context );
    
    // Clear all pending IRQ status bits
    lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
}

// Initialize the LR1121 radio module
void lora_radio_init( const void* context )
{
  // Retrieve the PA power configuration for the target frequency and power level
  const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr1121mb1gis_get_pa_pwr_cfg( RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

  if( pa_pwr_cfg == NULL )
  {
      printf( "Invalid target frequency or power level\n" );
      while( true )
      {
      }
  }

  // Print common configuration parameters
  printf( "Common parameters:\n" );
  printf( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( PACKET_TYPE ) );
  printf( "   RF frequency  = %u Hz\n", RF_FREQ_IN_HZ );
  printf( "   Output power  = %i dBm\n", TX_OUTPUT_POWER_DBM );
  printf( "   Fallback mode = %s\n", lr11xx_radio_fallback_modes_to_str( FALLBACK_MODE ) );
  printf( ( ENABLE_RX_BOOST_MODE == true ) ? "   Rx boost activated\n" : "   Rx boost deactivated\n" );
  printf( "\n" );

  // Set the packet type
  lr11xx_radio_set_pkt_type( context, PACKET_TYPE );

  // Verify the packet type setting
  lr11xx_radio_pkt_type_t spi_check;
  lr11xx_radio_get_pkt_type(context, &spi_check);
  if(spi_check == LR11XX_RADIO_PKT_TYPE_LORA)
  {
      printf("LoRa modulation\r\n" );
  }
  else if(spi_check == LR11XX_RADIO_PKT_TYPE_GFSK)
  {
      printf("GFSK modulation\r\n" );
  }
  else
    printf("spi_check_err\r\n" );

  // Set the RF frequency
  lr11xx_radio_set_rf_freq( context, RF_FREQ_IN_HZ );

  // Set the RSSI calibration table
  lr11xx_radio_set_rssi_calibration(context, smtc_shield_lr11xx_get_rssi_calibration_table( RF_FREQ_IN_HZ ));

  // Configure the PA settings
  lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) );

  // Set the TX power and ramp time
  lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME );

  // Set the fallback mode after TX/RX operations
  lr11xx_radio_set_rx_tx_fallback_mode( context, FALLBACK_MODE );
  // Configure the RX boost mode
  lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE );

  // Configure LoRa or GFSK parameters based on the packet type
  if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
  {
    print_lora_configuration( );
    lora_mod_params.ldro = smtc_shield_lr11xx_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );
    lr11xx_radio_set_lora_mod_params( context, &lora_mod_params );
    lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params );
    lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD );
  }
  // Configure the radio for GFSK modulation
  else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
  {
      // Print the current GFSK configuration
      print_gfsk_configuration( );

      // Set the GFSK modulation parameters
      lr11xx_radio_set_gfsk_mod_params( context, &gfsk_mod_params );
      // Set the GFSK packet parameters
      lr11xx_radio_set_gfsk_pkt_params( context, &gfsk_pkt_params );
      // Set the GFSK sync word
      lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word );

      // If DC-free encoding is enabled, set the whitening seed
      if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
      {
          lr11xx_radio_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED );
      }

      // If CRC is enabled, set the CRC parameters
      if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
      {
          lr11xx_radio_set_gfsk_crc_params( context, FSK_CRC_SEED, FSK_CRC_POLYNOMIAL );
      }

      // If address filtering is enabled, set the packet address
      if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
      {
          lr11xx_radio_set_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS );
      }
  }
  // Configure the radio for LR-FHSS modulation
  else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LR_FHSS )
  {
      // Define the LR-FHSS modulation parameters
      const lr11xx_radio_mod_params_lr_fhss_t mod_lr_fhss = {
          .br_in_bps   = LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS, // Bitrate in bps
          .pulse_shape = LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1, // Pulse shape
      };

      // Set the LR-FHSS modulation parameters
      lr11xx_radio_set_lr_fhss_mod_params( context, &mod_lr_fhss );
  }
}

void lora_radio_dbpsk_init( const void* context, const uint8_t payload_len )
{
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr1121mb1gis_get_pa_pwr_cfg( SIGFOX_UPLINK_RF_FREQ_IN_HZ, SIGFOX_TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        printf( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    printf( "Sigfox parameters:\n" );
    printf( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( LR11XX_RADIO_PKT_TYPE_BPSK ) );
    printf( "   RF frequency  = %u Hz\n", SIGFOX_UPLINK_RF_FREQ_IN_HZ );
    printf( "   Output power  = %i dBm\n", SIGFOX_TX_OUTPUT_POWER_DBM );

    lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_BPSK );
    lr11xx_radio_set_rf_freq( context, SIGFOX_UPLINK_RF_FREQ_IN_HZ );
    lr11xx_radio_set_rssi_calibration(
        context, smtc_shield_lr11xx_get_rssi_calibration_table( SIGFOX_UPLINK_RF_FREQ_IN_HZ ) );
    lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) );

    lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME ) ;

    lr11xx_radio_set_bpsk_mod_params( context, &bpsk_mod_params );

    bpsk_pkt_params.pld_len_in_bytes = smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 );
    bpsk_pkt_params.pld_len_in_bits  = smtc_dbpsk_get_pld_len_in_bits( payload_len << 3 );

    if( BPSK_BITRATE_IN_BPS == 100 )
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS;
    }
    else if( BPSK_BITRATE_IN_BPS == 600 )
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS;
    }
    else
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_DEFAULT;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_DEFAULT;
    }

    lr11xx_radio_set_bpsk_pkt_params( context, &bpsk_pkt_params );
}


// Print the LoRa configuration parameters
void print_lora_configuration(void)
{
    // Print LoRa modulation parameters
    printf( "LoRa modulation parameters:\n" );
    printf( "   Spreading factor = %s\n", lr11xx_radio_lora_sf_to_str( LORA_SPREADING_FACTOR ) ); // Spreading factor
    printf( "   Bandwidth        = %s\n", lr11xx_radio_lora_bw_to_str( LORA_BANDWIDTH ) ); // Bandwidth
    printf( "   Coding rate      = %s\n", lr11xx_radio_lora_cr_to_str( LORA_CODING_RATE ) ); // Coding rate
    printf( "\n" );

    // Print LoRa packet parameters
    printf( "LoRa packet parameters:\n" );
    printf( "   Preamble length = %d symbol(s)\n", LORA_PREAMBLE_LENGTH ); // Preamble length in symbols
    printf( "   Header mode     = %s\n", lr11xx_radio_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) ); // Header mode
    printf( "   Payload length  = %d byte(s)\n", PAYLOAD_LENGTH ); // Payload length in bytes
    printf( "   CRC mode        = %s\n", lr11xx_radio_lora_crc_to_str( LORA_CRC ) ); // CRC mode
    printf( "   IQ              = %s\n", lr11xx_radio_lora_iq_to_str( LORA_IQ ) ); // IQ inversion
    printf( "\n" );

    // Print LoRa syncword
    printf( "LoRa syncword = 0x%02X\n", LORA_SYNCWORD );
    printf( "\n" );
}

// Print the GFSK configuration parameters
void print_gfsk_configuration( void )
{
    // Print GFSK modulation parameters
    printf( "GFSK modulation parameters:\n" );
    printf( "   Bitrate             = %u bps\n", FSK_BITRATE ); // Bitrate in bps
    printf( "   Pulse shape         = %s\n", lr11xx_radio_gfsk_pulse_shape_to_str( FSK_PULSE_SHAPE ) ); // Pulse shape
    printf( "   Bandwidth           = %s\n", lr11xx_radio_gfsk_bw_to_str( FSK_BANDWIDTH ) ); // Bandwidth
    printf( "   Frequency deviation = %u Hz\n", FSK_FDEV ); // Frequency deviation in Hz
    printf( "\n" );

    // Print GFSK packet parameters
    printf( "GFSK packet parameters:\n" );
    printf( "   Preamble length   = %d bit(s)\n", FSK_PREAMBLE_LENGTH ); // Preamble length in bits
    printf( "   Preamble detector = %s\n", lr11xx_radio_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) ); // Preamble detector
    printf( "   Syncword length   = %d bit(s)\n", FSK_SYNCWORD_LENGTH ); // Syncword length in bits
    printf( "   Address filtering = %s\n", lr11xx_radio_gfsk_address_filtering_to_str( FSK_ADDRESS_FILTERING ) ); // Address filtering mode
    if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
    {
        printf( "     (Node address      = 0x%02X)\n", FSK_NODE_ADDRESS ); // Node address
        if( FSK_ADDRESS_FILTERING == LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES )
        {
            printf( "     (Broadcast address = 0x%02X)\n", FSK_BROADCAST_ADDRESS ); // Broadcast address
        }
    }
    printf( "   Header mode       = %s\n", lr11xx_radio_gfsk_pkt_len_modes_to_str( FSK_HEADER_TYPE ) ); // Header mode
    printf( "   Payload length    = %d byte(s)\n", PAYLOAD_LENGTH ); // Payload length in bytes
    printf( "   CRC mode          = %s\n", lr11xx_radio_gfsk_crc_type_to_str( FSK_CRC_TYPE ) ); // CRC mode
    if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
    {
        printf( "     (CRC seed       = 0x%08X)\n", FSK_CRC_SEED ); // CRC seed
        printf( "     (CRC polynomial = 0x%08X)\n", FSK_CRC_POLYNOMIAL ); // CRC polynomial
    }
    printf( "   DC free           = %s\n", lr11xx_radio_gfsk_dc_free_to_str( FSK_DC_FREE ) ); // DC-free encoding mode
    if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
    {
        printf( "     (Whitening seed = 0x%04X)\n", FSK_WHITENING_SEED ); // Whitening seed
    }
    printf( "\n" );
}

// Calculate the time on air for the configured packet
uint32_t get_time_on_air_in_ms( void )
{
    // Determine the time on air based on the packet type
    switch( PACKET_TYPE )
    {
      case LR11XX_RADIO_PKT_TYPE_LORA:
      {
          // Calculate time on air for LoRa
          return lr11xx_radio_get_lora_time_on_air_in_ms( &lora_pkt_params, &lora_mod_params );
      }
      case LR11XX_RADIO_PKT_TYPE_GFSK:
      {
          // Calculate time on air for GFSK
          return lr11xx_radio_get_gfsk_time_on_air_in_ms( &gfsk_pkt_params, &gfsk_mod_params );
      }
      default:
      {
          // Return 0 if the packet type is not recognized
          return 0;
      }
    }
}