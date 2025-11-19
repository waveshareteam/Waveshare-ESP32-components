/*!
 * @file      lr1121_modem_printf_info.c
 *
 * @brief     Common Application Helper function implementations
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1121_modem_printf_info.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void print_hex_buffer(const uint8_t *buffer, uint8_t size)
{
    uint8_t newline = 0;

    for (uint8_t i = 0; i < size; i++)
    {
        if (newline != 0)
        {
            printf("\n\n");
            newline = 0;
        }

        printf("%02X ", buffer[i]);

        if (((i + 1) % 16) == 0)
        {
            newline = 1;
        }
    }
    printf("\n\n");
}

void print_version(lr1121_modem_version_t modem_version)
{
    printf("###### ===== lr1121 MODEM-E VERSION ==== ######\n\n\n");
    printf("USE CASE   : %02X\n", modem_version.use_case);
    printf("MODEM      : %02X.%02X.%02X\n", modem_version.modem_major, modem_version.modem_minor,
           modem_version.modem_patch);
    printf("LBM        : %02X.%02X.%02X\n\n", modem_version.lbm_major, modem_version.lbm_minor,
           modem_version.lbm_patch);
}

void print_lorawan_credentials(const uint8_t *dev_eui, const uint8_t *join_eui, const uint8_t *pin,
                               const bool use_internal_credentials)
{
    if (use_internal_credentials)
    {
        printf("---=== INTERNAL CREDENTIALS ===---\n\n");
    }
    else
    {
        printf("---=== CUSTOM CREDENTIALS ===---\n\n");
    }
    printf("DevEui      : %02X", dev_eui[0]);
    for (int i = 1; i < 8; i++)
    {
        printf("-%02X", dev_eui[i]);
    }
    printf("\n");
    printf("JoinEui     : %02X", join_eui[0]);
    for (int i = 1; i < 8; i++)
    {
        printf("-%02X", join_eui[i]);
    }
    printf("\n");
    printf("Pin         : %02X", pin[0]);
    for (int i = 1; i < 4; i++)
    {
        printf("-%02X", pin[i]);
    }
    printf("\n\n");
}

void modem_status_to_string(lr1121_modem_lorawan_status_t modem_status)
{
    printf("Modem status : ");

    if ((modem_status & LR1121_LORAWAN_CRASH) == LR1121_LORAWAN_CRASH)
    {
        printf("CRASH ");
    }
    if ((modem_status & LR1121_LORAWAN_JOINED) == LR1121_LORAWAN_JOINED)
    {
        printf("JOINED ");
    }
    if ((modem_status & LR1121_LORAWAN_SUSPEND) == LR1121_LORAWAN_SUSPEND)
    {
        printf("SUSPEND ");
    }
    if ((modem_status & LR1121_LORAWAN_JOINING) == LR1121_LORAWAN_JOINING)
    {
        printf("JOINING ");
    }

    printf("\n\n\n");
}

void get_and_print_lorawan_region_from_modem(const void *context, lr1121_modem_regions_t *modem_region)
{
    // 1. Get the region from modem
    lr1121_modem_regions_t local_region = LR1121_LORAWAN_REGION_EU868;
    const lr1121_modem_response_code_t response_code = lr1121_modem_get_region(context, &local_region);
    if (response_code == LR1121_MODEM_RESPONSE_CODE_OK)
    {
        // 2a. If the get from the modem is successful print it.
        //     And the output pointer is not null: return the region
        if (modem_region != NULL)
        {
            *modem_region = local_region;
        }
        print_lorawan_region(*modem_region);
    }
    else
    {
        // 2b. If the get from modem failed: print an error message
        printf("Error on lr1121_modem_get_region, get response code: %d\n", response_code);
    }
}

void print_lorawan_region(lr1121_modem_regions_t region)
{
    switch (region)
    {
    case LR1121_LORAWAN_REGION_EU868:
    {
        printf("REGION      : EU868\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_US915:
    {
        printf("REGION      : US915\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_AU915:
    {
        printf("REGION      : AU915\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_AS923_GRP1:
    {
        printf("REGION      : AS923_GRP1\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_CN470:
    {
        printf("REGION      : CN470\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_AS923_GRP2:
    {
        printf("REGION      : AS923_GRP2\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_AS923_GRP3:
    {
        printf("REGION      : AS923_GRP3\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_AS923_GRP4:
    {
        printf("REGION      : AS923_GRP4\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_IN865:
    {
        printf("REGION      : IN865\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_KR920:
    {
        // printf( "LBT         : ACTIVATE LBT\n\n" );
        printf("REGION      : KR920\n\n\n");
        break;
    }
    case LR1121_LORAWAN_REGION_RU864:
    {
        printf("REGION      : RU864\n\n\n");
        break;
    }
    default:
        printf("No supported region selected\n\n\n");
        break;
    }
}

void print_certification(lr1121_modem_certification_mode_t certif_running)
{
    if (certif_running == LR1121_MODEM_CERTIFICATION_MODE_ENABLE)
    {
        printf("###### ===================================== ######\n");
        printf("###### ===== CERTIFICATION MODE ENABLED ==== ######\n");
        printf("###### ===================================== ######\n\n\n");
    }
    else
    {
        printf("###### ====================================== ######\n");
        printf("###### ===== CERTIFICATION MODE DISABLED ==== ######\n");
        printf("###### ====================================== ######\n\n\n");
    }
}

void get_and_print_crashlog(const void *context)
{
    lr1121_modem_response_code_t response_code = LR1121_MODEM_RESPONSE_CODE_OK;
    lr1121_modem_lorawan_status_t modem_status;
    
    response_code = lr1121_modem_get_status(context, (lr1121_modem_lorawan_status_bitmask_t*)&modem_status);
    // Check if the crashlog bit is set in modem_status
    if (response_code == LR1121_MODEM_RESPONSE_CODE_OK)
    {
        if ((modem_status & LR1121_LORAWAN_CRASH) == LR1121_LORAWAN_CRASH)
        {
            lr1121_modem_crashlog_status_t status_crashlog = LR1121_NO_NEW_CRASHLOG;
            uint8_t crashlog[242] = {0};
            // Get the crashlog
            response_code = lr1121_modem_get_crashlog(context, &status_crashlog, crashlog);
            if ((response_code == LR1121_MODEM_RESPONSE_CODE_OK) && (status_crashlog == LR1121_NEW_CRASHLOG))
            {
                printf("###### ===================================== ######\n");
                printf("###### =========== MODEM CRASHED =========== ######\n");
                printf("###### ===================================== ######\n\n");
                //  HAL_DBG_TRACE_ARRAY( "Crashlog: ", crashlog, 242 );
                printf("%s - (%u bytes):\n", "Crashlog: ", 242);
                for (uint32_t i = 0; i < 242; i++)
                {
                    if (((i % 16) == 0) && (i > 0))
                    {
                        printf("\n");
                    }
                    printf(" %02X", crashlog[i]);
                }
                printf("\n");
                printf("\n\n");
            }
        }
    }
    else
    {
        printf("Error on lr1121_modem_get_status, get response code: %d\n", response_code);
    }
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
