/*!
 * @file      lr1121_modem_lorawan_types.h
 *
 * @brief     LoRaWAN driver types for LR1121 modem
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

#ifndef LR1121_MODEM_LORAWAN_TYPES_H
#define LR1121_MODEM_LORAWAN_TYPES_H

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
 * @brief Length in bytes of a LoRaWAN device eui
 */
#define LR1121_MODEM_DEV_EUI_LENGTH ( 8 )

/*!
 * @brief Length in bytes of a LoRaWAN join eui
 */
#define LR1121_MODEM_JOIN_EUI_LENGTH ( 8 )

/*!
 * @brief Length in bytes of a LoRaWAN application key
 */
#define LR1121_MODEM_APP_KEY_LENGTH ( 16 )

/*!
 * @brief Length in bytes of a LoRaWAN network key
 */
#define LR1121_MODEM_NWK_KEY_LENGTH ( 16 )

/*!
 * @brief Length of datarate distribution array
 */
#define LR1121_MODEM_DATARATE_DISTRIBUTION_LENGTH ( 16 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief LoRaWAN class type
 */
typedef enum
{
    LR1121_LORAWAN_CLASS_A = 0x00,
    LR1121_LORAWAN_CLASS_B = 0x01,
    LR1121_LORAWAN_CLASS_C = 0x02,
} lr1121_modem_classes_t;

/**
 * @brief LoRaWAN network type
 */
typedef enum
{
    LR1121_MODEM_LORAWAN_PRIVATE_NETWORK = 0x00,  //!< LoRaWAN private network
    LR1121_MODEM_LORAWAN_PUBLIC_NETWORK  = 0x01   //!< LoRaWAN public network
} lr1121_modem_network_type_t;

/*!
 * @brief LoRaWAN region type
 */
typedef enum
{
    LR1121_LORAWAN_REGION_EU868        = 0x01,
    LR1121_LORAWAN_REGION_AS923_GRP1   = 0x02,
    LR1121_LORAWAN_REGION_US915        = 0x03,
    LR1121_LORAWAN_REGION_AU915        = 0x04,
    LR1121_LORAWAN_REGION_CN470        = 0x05,
    LR1121_LORAWAN_REGION_WW2G4        = 0x06,
    LR1121_LORAWAN_REGION_AS923_GRP2   = 0x07,
    LR1121_LORAWAN_REGION_AS923_GRP3   = 0x08,
    LR1121_LORAWAN_REGION_IN865        = 0x09,
    LR1121_LORAWAN_REGION_KR920        = 0x0A,
    LR1121_LORAWAN_REGION_RU864        = 0x0B,
    LR1121_LORAWAN_REGION_CN470_RP_1_0 = 0x0C,
    LR1121_LORAWAN_REGION_AS923_GRP4   = 0x0D,
} lr1121_modem_regions_t;

/*!
 * @brief Adaptative Data Rate profiles type
 */
typedef enum
{
    LR1121_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED = 0x00,  //!< Network Server Controlled
    LR1121_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE = 0x01,  //!< Mobile Long Range : 50% MinDr, 25% MinDr + 1, 25% MinDr + 2
    LR1121_MODEM_ADR_PROFILE_MOBILE_LOW_POWER =
        0x02,  //!< Mobile Low Power : 25% MaxDr, 25% MaxDr - 1, 25% MaxDr - 2, 25% MaxDr - 3
    LR1121_MODEM_ADR_PROFILE_CUSTOM =
        0x03,  //!< Custom List A custom ADR profile consists of a list of 16 preferred data rates.
               //!< For every transmission, a random entry in that list is selected.
} lr1121_modem_adr_profiles_t;

/*!
 * @brief LoRaWAN uplink type
 */
typedef enum
{
    LR1121_MODEM_UPLINK_UNCONFIRMED = 0x00,
    LR1121_MODEM_UPLINK_CONFIRMED   = 0x01,
} lr1121_modem_uplink_type_t;

/*!
 * @brief LoRaWAN Duty Cycle activation type
 */
typedef enum
{
    LR1121_MODEM_CERTIFICATION_MODE_DISABLE = 0x00,
    LR1121_MODEM_CERTIFICATION_MODE_ENABLE  = 0x01,
} lr1121_modem_certification_mode_t;

/*!
 * @brief Listen Before Talk (LBT) activation type
 */
typedef enum
{
    LR1121_MODEM_LBT_MODE_DISABLE = 0x00,
    LR1121_MODEM_LBT_MODE_ENABLE  = 0x01,
} lr1121_modem_lbt_mode_t;

/*!
 * @brief Carrier Sense Multiple Access (CSMA) activation type
 */
typedef enum
{
    LR1121_MODEM_CSMA_MODE_DISABLE = 0x00,
    LR1121_MODEM_CSMA_MODE_ENABLE  = 0x01,
} lr1121_modem_csma_mode_t;

/**
 * @brief Carrier Sense Multiple Access (CSMA) backoff activation type
 */
typedef enum
{
    LR1121_MODEM_CSMA_BACKOFF_DISABLE = 0x00,  //!< Disable CMSA backoff
    LR1121_MODEM_CSMA_BACKOFF_ENABLE  = 0x01,  //!< Enable CMSA backoff
} lr1121_modem_csma_backoff_mode_t;

/*!
 * @brief LoRaWAN mac request field
 */
typedef enum
{
    LR1121_MODEM_MAC_REQUEST_LINK_CHECK     = 0x01,  //!< Enable the MAC Link Check request
    LR1121_MODEM_MAC_REQUEST_TIME           = 0x02,  //!< Enable the MAC Time request
    LR1121_MODEM_MAC_REQUEST_PING_SLOT_INFO = 0x04,  //!< Enable the MAC Ping Slot Info request
} lr1121_modem_mac_request_t;

/**
 * @brief Bit mask for lr1121_modem_mac_request_t
 *
 * @see lr1121_modem_mac_request_t
 */
typedef uint8_t lr1121_modem_mac_request_bitmask_t;

/*!
 * @brief RX flags encoding
 */
typedef enum
{
    LR1121_MODEM_DOWNLINK_WINDOW_RX1                  = 0x01,  //!< received on RX1 unicast
    LR1121_MODEM_DOWNLINK_WINDOW_RX2                  = 0x02,  //!< received on RX2 unicast
    LR1121_MODEM_DOWNLINK_WINDOW_RXC                  = 0x03,  //!< received on Class C RX unicast
    LR1121_MODEM_DOWNLINK_WINDOW_RXC_MULTICAST_GROUP0 = 0x04,  //!< received on Class C Multicast RX for group 0
    LR1121_MODEM_DOWNLINK_WINDOW_RXC_MULTICAST_GROUP1 = 0x05,  //!< received on Class C Multicast RX for group 1
    LR1121_MODEM_DOWNLINK_WINDOW_RXC_MULTICAST_GROUP2 = 0x06,  //!< received on Class C Multicast RX for group 2
    LR1121_MODEM_DOWNLINK_WINDOW_RXC_MULTICAST_GROUP3 = 0x07,  //!< received on Class C Multicast RX for group 3
    LR1121_MODEM_DOWNLINK_WINDOW_RXB                  = 0x08,  //!< received on Class B RX unicast
    LR1121_MODEM_DOWNLINK_WINDOW_RXB_MULTICAST_GROUP0 = 0x09,  //!< received on Class B Multicast RX for group 0
    LR1121_MODEM_DOWNLINK_WINDOW_RXB_MULTICAST_GROUP1 = 0x0A,  //!< received on Class B Multicast RX for group 1
    LR1121_MODEM_DOWNLINK_WINDOW_RXB_MULTICAST_GROUP2 = 0x0B,  //!< received on Class B Multicast RX for group 2
    LR1121_MODEM_DOWNLINK_WINDOW_RXB_MULTICAST_GROUP3 = 0x0C,  //!< received on Class B Multicast RX for group 3
    LR1121_MODEM_DOWNLINK_WINDOW_RXBEACON             = 0x0D,  //!< received a Class B beacon
    LR1121_MODEM_DOWNLINK_WINDOW_RXRELAY              = 0x0E,  //!< received on Relay window
} lr1121_modem_downlink_window_t;

/*!
 * @brief class b ping slot status
 */
typedef enum
{
    LR1121_MODEM_CLASS_B_PING_SLOT_1_S   = 0x00,  //!< 1 second ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_2_S   = 0x01,  //!< 2 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_4_S   = 0x02,  //!< 4 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_8_S   = 0x03,  //!< 8 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_16_S  = 0x04,  //!< 16 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_32_S  = 0x05,  //!< 32 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_64_S  = 0x06,  //!< 64 seconds ping-slot periodicity
    LR1121_MODEM_CLASS_B_PING_SLOT_128_S = 0x07,  //!< 128 seconds ping-slot periodicity
} lr1121_modem_class_b_ping_slot_t;

/*!
 * @brief Select the source of battery level value to use for DevStatusAns MAC command
 *
 */
typedef enum
{
    LR1121_MODEM_BATTERY_LEVEL_FROM_INTERNAL_VALUE =
        0x00,  //!< Battery level is obtained from LR1121 VBat (see @ref lr1121_modem_system_get_vbat)
    LR1121_MODEM_BATTERY_LEVEL_FROM_USER_VALUE = 0x01,  //!< Battery level is provided by user application
} lr1121_modem_battery_level_source_value_t;

/*!
 * @brief Join EUI type
 */
typedef uint8_t lr1121_modem_join_eui_t[LR1121_MODEM_JOIN_EUI_LENGTH];

/*!
 * @brief Device EUI type
 */
typedef uint8_t lr1121_modem_dev_eui_t[LR1121_MODEM_DEV_EUI_LENGTH];

/*!
 * @brief Application key type
 */
typedef uint8_t lr1121_modem_app_key_t[LR1121_MODEM_APP_KEY_LENGTH];

/*!
 * @brief Application key type
 */
typedef uint8_t lr1121_modem_nwk_key_t[LR1121_MODEM_NWK_KEY_LENGTH];

/*!
 * @brief modem downlink metadata structure
 *
 * The Signal to Noise Ratio (SNR) is returned as an integer part, and a decimal part in 0.25 dB.
 * The SNR in dB is therefore obtained by: \f$ SNR_{dB} = snr\_integer + 0.25 \times snr\_quarter \f$
 */
typedef struct
{
    uint8_t stack_id;                       //!< The stack identifier that receives the downlink
    int16_t rssi;                           //!< The RSSI of the received downlink in dBm
    int8_t  snr_integer;                    //!< Signal to Noise Ratio of the received downlink (integer part, in dB)
    uint8_t snr_quarter;                    //!< Signal to Noise Ratio of the received downlink (0.25dB counts)
    lr1121_modem_downlink_window_t window;  //!< Rx window of the received downlink
    uint8_t                        fport;   //!< LoRaWAN port of the received downlink
    uint8_t                        fpending_bit;  //!< Frame pending bit of the received downlink
    uint32_t                       frequency_hz;  //!< RF frequency of the received downlink
    uint8_t                        datarate;      //!< Datarate of the received downlink
} lr1121_modem_downlink_metadata_t;

/*!
 * @brief multicast class c status structure
 */
typedef struct
{
    uint8_t  is_session_started;  //!< Indicate if the multicast class C session is started
    uint32_t downlink_frequency;  //!< Downlink frequency of the multicast class C session
    uint8_t  downlink_datarate;   //!< Datarate of the multicast class C session
} lr1121_modem_multicast_class_c_status_t;

/*!
 * @brief multicast class b status structure
 */
typedef struct
{
    uint8_t  is_session_started;             //!< Indicate if the multicast class B session is started
    uint32_t downlink_frequency;             //!< Downlink frequency of the multicast class B session
    uint8_t  downlink_datarate;              //!< Datarate of the multicast class B session
    uint8_t  is_session_waiting_for_beacon;  //!< Indicates whether the multicast class B session have received a beacon
    uint8_t  ping_slot_periodicity;          //!< Ping slot periodicity of the multicast class B session
} lr1121_modem_multicast_class_b_status_t;

/*!
 * @brief LR1121 LoRaWAN version structure
 */
typedef struct
{
    uint8_t lorawan_major;     //!< Major number of the LoRaWAN standard implemented
    uint8_t lorawan_minor;     //!< Minor number of the LoRaWAN standard implemented
    uint8_t lorawan_patch;     //!< Patch number of the LoRaWAN standard implemented
    uint8_t lorawan_revision;  //!< Revision number of the LoRaWAN standard implemented
    uint8_t rp_major;          //!< Major number of the regional parameters implemented
    uint8_t rp_minor;          //!< Minor number of the regional parameters implemented
    uint8_t rp_patch;          //!< Patch number of the regional parameters implemented
    uint8_t rp_revision;       //!< Revision number of the regional parameters implemented
} lr1121_modem_lorawan_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1121_MODEM_LORAWAN_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
