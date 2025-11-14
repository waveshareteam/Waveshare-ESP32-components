#pragma once

#include <stdint.h>
#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief LCD panel initialization commands.
     *
     */
    typedef struct
    {
        int cmd;               /*<! The specific LCD command */
        const void *data;      /*<! Buffer that holds the command specific data */
        size_t data_bytes;     /*<! Size of `data` in memory, in bytes */
        unsigned int delay_ms; /*<! Delay in milliseconds after this command */
    } dsi_lcd_init_cmd_t;

    /**
     * @brief LCD panel vendor configuration.
     *
     * @note  This structure needs to be passed to the `vendor_config` field in `esp_lcd_panel_dev_config_t`.
     *
     */
    typedef struct
    {
        const dsi_lcd_init_cmd_t *init_cmds; /*!< Pointer to initialization commands array. Set to NULL if using default commands.
                                                 *   The array should be declared as `static const` and positioned outside the function.
                                                 *   Please refer to `vendor_specific_init_default` in source file.
                                                 */
        uint16_t init_cmds_size;                /*<! Number of commands in above array */
        struct
        {
            esp_lcd_dsi_bus_handle_t dsi_bus;             /*!< MIPI-DSI bus configuration */
            const esp_lcd_dpi_panel_config_t *dpi_config; /*!< MIPI-DPI panel configuration */
        } mipi_config;
    } dsi_vendor_config_t;

    /**
     * @brief Create LCD panel for model Waveshare DSI Display
     *
     * @note  Vendor specific initialization can be different between manufacturers, should consult the LCD supplier for initialization sequence code.
     *
     * @param[in]  io LCD panel IO handle
     * @param[in]  panel_dev_config General panel device configuration
     * @param[out] ret_panel Returned LCD panel handle
     * @return
     *      - ESP_ERR_INVALID_ARG   if parameter is invalid
     *      - ESP_OK                on success
     *      - Otherwise             on fail
     */
    esp_err_t esp_lcd_new_panel_dsi(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                       esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief MIPI-DSI bus configuration structure
 *
 * @param[in] lane_num Number of data lanes
 * @param[in] lane_mbps Lane bit rate in Mbps
 *
 */
#define DSI_PANEL_BUS_DSI_2CH_CONFIG()            \
    {                                                \
        .bus_id = 0,                                 \
        .num_data_lanes = 2,                         \
        .phy_clk_src = 0, \
        .lane_bit_rate_mbps = 1250,                  \
    }

/**
 * @brief MIPI-DBI panel IO configuration structure
 *
 */
#define DSI_PANEL_IO_DBI_CONFIG() \
    {                                \
        .virtual_channel = 0,        \
        .lcd_cmd_bits = 8,           \
        .lcd_param_bits = 8,         \
    }

/**
 * @brief MIPI DPI configuration structure
 *
 * @note  refresh_rate = (dpi_clock_freq_mhz * 1000000) / (h_res + hsync_pulse_width + hsync_back_porch + hsync_front_porch)
 *                                                      / (v_res + vsync_pulse_width + vsync_back_porch + vsync_front_porch)
 *
 * @param[in] px_format Pixel format of the panel
 *
 */
#define DSI_PANEL_DPI_2_8_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 480,                               \
            .v_size = 640,                               \
            .hsync_back_porch = 150,                     \
            .hsync_pulse_width = 150,                    \
            .hsync_front_porch = 50,                     \
            .vsync_back_porch = 50,                      \
            .vsync_pulse_width = 150,                    \
            .vsync_front_porch = 150,                    \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_3_4_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 800,                               \
            .v_size = 800,                               \
            .hsync_back_porch = 40,                      \
            .hsync_pulse_width = 6,                      \
            .hsync_front_porch = 120,                    \
            .vsync_back_porch = 12,                      \
            .vsync_pulse_width = 4,                      \
            .vsync_front_porch = 16,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_4_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 720,                               \
            .hsync_back_porch = 32,                      \
            .hsync_pulse_width = 200,                    \
            .hsync_front_porch = 120,                    \
            .vsync_back_porch = 4,                       \
            .vsync_pulse_width = 16,                     \
            .vsync_front_porch = 8,                      \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_4_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48.6,                      \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 480,                               \
            .v_size = 800,                               \
            .hsync_back_porch = 150,                     \
            .hsync_pulse_width = 100,                    \
            .hsync_front_porch = 150,                    \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 100,                    \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_5_INCH_D_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 1280,                              \
            .hsync_back_porch = 100,                     \
            .hsync_pulse_width = 100,                    \
            .hsync_front_porch = 80,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 20,                     \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_6_25_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 720,                               \
            .v_size = 1560,                              \
            .hsync_back_porch = 50,                      \
            .hsync_pulse_width = 50,                     \
            .hsync_front_porch = 50,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 20,                     \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_5_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1024,                              \
            .v_size = 600,                               \
            .hsync_back_porch = 100,                     \
            .hsync_pulse_width = 100,                    \
            .hsync_front_porch = 100,                    \
            .vsync_back_porch = 10,                      \
            .vsync_pulse_width = 10,                     \
            .vsync_front_porch = 10,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_7_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1024,                              \
            .v_size = 600,                               \
            .hsync_back_porch = 100,                     \
            .hsync_pulse_width = 100,                    \
            .hsync_front_porch = 100,                    \
            .vsync_back_porch = 10,                      \
            .vsync_pulse_width = 10,                     \
            .vsync_front_porch = 10,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_7_9_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 400,                               \
            .v_size = 1280,                              \
            .hsync_back_porch = 40,                      \
            .hsync_pulse_width = 30,                     \
            .hsync_front_porch = 40,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 10,                     \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_7_INCH_E_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1280,                              \
            .v_size = 800,                               \
            .hsync_back_porch = 156,                     \
            .hsync_pulse_width = 40,                     \
            .hsync_front_porch = 20,                     \
            .vsync_back_porch = 48,                      \
            .vsync_pulse_width = 40,                     \
            .vsync_front_porch = 40,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_7_INCH_H_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1280,                              \
            .v_size = 720,                               \
            .hsync_back_porch = 64,                     \
            .hsync_pulse_width = 64,                     \
            .hsync_front_porch = 64,                     \
            .vsync_back_porch = 64,                      \
            .vsync_pulse_width = 64,                     \
            .vsync_front_porch = 64,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_8_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1280,                              \
            .v_size = 800,                               \
            .hsync_back_porch = 156,                     \
            .hsync_pulse_width = 40,                     \
            .hsync_front_porch = 20,                     \
            .vsync_back_porch = 48,                      \
            .vsync_pulse_width = 40,                     \
            .vsync_front_porch = 40,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_10_1_INCH_C_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 80,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 1280,                              \
            .v_size = 800,                               \
            .hsync_back_porch = 156,                     \
            .hsync_pulse_width = 40,                     \
            .hsync_front_porch = 20,                     \
            .vsync_back_porch = 48,                      \
            .vsync_pulse_width = 40,                     \
            .vsync_front_porch = 40,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_8_8_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 480,                               \
            .v_size = 1920,                              \
            .hsync_back_porch = 50,                      \
            .hsync_pulse_width = 50,                     \
            .hsync_front_porch = 50,                     \
            .vsync_back_porch = 20,                      \
            .vsync_pulse_width = 20,                     \
            .vsync_front_porch = 20,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#define DSI_PANEL_DPI_11_9_INCH_CONFIG(px_format) \
    {                                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     \
        .dpi_clock_freq_mhz = 48,                        \
        .virtual_channel = 0,                            \
        .pixel_format = px_format,                       \
        .num_fbs = 1,                                    \
        .video_timing = {                                \
            .h_size = 320,                               \
            .v_size = 1480,                              \
            .hsync_back_porch = 60,                      \
            .hsync_pulse_width = 60,                     \
            .hsync_front_porch = 60,                     \
            .vsync_back_porch = 60,                      \
            .vsync_pulse_width = 60,                     \
            .vsync_front_porch = 60,                     \
        },                                               \
        .flags.use_dma2d = true,                         \
    }
#endif

#ifdef __cplusplus
}
#endif
