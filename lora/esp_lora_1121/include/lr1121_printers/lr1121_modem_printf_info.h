/**
 * @file      lr1121_modem_printf_info.h
 *
 * @brief     Common Application Helper functions
 *
 * @copyright
 * @parblock
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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
 * @endparblock
 */
#ifndef LR1121_MODEM_PRINTF_INFO_H
#define LR1121_MODEM_PRINTF_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "lr1121_modem/lr1121_modem_lorawan.h"
#include "lr1121_modem/lr1121_modem_modem.h"

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

/**
 * @brief Prints the provided buffer in HEX
 *
 * @param [in] buffer Buffer to be printed
 * @param [in] size Buffer size to be printed
 */
void print_hex_buffer( const uint8_t* buffer, uint8_t size );

/**
 * @brief Prints the LoRaWAN keys
 *
 * @param [in] dev_eui Device EUI to be printed
 * @param [in] join_eui Join EUI to be printed
 * @param [in] use_internal_credentials specify if the internal credentials are used
 */
void print_lorawan_credentials( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* pin,
                                const bool use_internal_credentials );

/**
 * @brief Prints the LoRaWAN version
 *
 * @param [in] modem_version Modem version to be printed
 */
void print_version( lr1121_modem_version_t modem_version );

/**
 * @brief convert lr1121 modem-e status to string
 */
void modem_status_to_string( lr1121_modem_lorawan_status_t modem_status );

/**
 * @brief Get the lorawan region from modem and print it
 *
 * @param [in] context Chip implementation context
 * @param [out] modem_region The LoRaWAN region returned by the modem. This pointer can be NULL: in this case the region
 * is only printed, and not returned to caller
 */
void get_and_print_lorawan_region_from_modem( const void* context, lr1121_modem_regions_t* modem_region );

/**
 * @brief Prints the LoRaWAN region
 *
 * @param [in] region Region to be printed
 */
void print_lorawan_region( lr1121_modem_regions_t region );

/**
 * @brief Prints the state of certification mode
 *
 * @param [in] certif_running State of certification mode
 */
void print_certification( lr1121_modem_certification_mode_t certif_running );

/**
 * @brief Gets and prints the crashlog if the crashlog status bit is set
 *
 * @param [in] context Chip implementation context
 */
void get_and_print_crashlog( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // APPS_UTILITIES_H

/* --- EOF ------------------------------------------------------------------ */
