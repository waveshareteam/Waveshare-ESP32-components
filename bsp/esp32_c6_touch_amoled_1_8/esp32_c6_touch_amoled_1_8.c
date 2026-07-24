/*
 * SPDX-FileCopyrightText: 2026 Waveshare Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_co5300.h"
#include "esp_lcd_sh8601.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_touch_ft5x06.h"

#include "esp_codec_dev_defaults.h"
#include "bsp/esp32_c6_touch_amoled_1_8.h"
#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp/touch.h"

static const char *TAG = "ESP32-C6-Touch-AMOLED-1.8";

static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;
static esp_io_expander_handle_t io_expander = NULL;
static bool display_touch_reset_done = false;
static bsp_board_variant_t board_variant = BSP_BOARD_VARIANT_UNKNOWN;
static lv_indev_t *disp_indev = NULL;
sdmmc_card_t *bsp_sdcard = NULL;
static esp_lcd_touch_handle_t tp = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static uint8_t brightness = 0;
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;
static bool lcd_spi_initialized = false;

#define BSP_TOUCH_RESET_DELAY_MS (200)
#define BSP_LCD_CO5300_X_GAP (0x10)

#define BSP_I2S_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S_MCLK,  \
        .bclk = BSP_I2S_SCLK,  \
        .ws = BSP_I2S_LCLK,    \
        .dout = BSP_I2S_DOUT,  \
        .din = BSP_I2S_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

static const sh8601_lcd_init_cmd_t sh8601_lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x44, (uint8_t[]){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
};

static const co5300_lcd_init_cmd_t co5300_lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
    {0x11, (uint8_t[]){0x00}, 0, 100},
    {0x29, (uint8_t[]){0x00}, 0, 0},
};

#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

/**************************************************************************************************
 *
 * I2C Function
 *
 **************************************************************************************************/
esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .trans_queue_depth = 0,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    if (!i2c_initialized) {
        return ESP_OK;
    }

    if (io_expander != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_del(io_expander));
        io_expander = NULL;
    }
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_handle = NULL;
    i2c_initialized = false;
    display_touch_reset_done = false;
    board_variant = BSP_BOARD_VARIANT_UNKNOWN;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    if (bsp_i2c_init() != ESP_OK) {
        return NULL;
    }
    return i2c_handle;
}

static esp_err_t bsp_i2c_device_probe(uint8_t addr)
{
    return i2c_master_probe(i2c_handle, addr, 100);
}

static esp_err_t bsp_display_touch_reset(void)
{
    if (display_touch_reset_done) {
        return ESP_OK;
    }

    esp_io_expander_handle_t expander = bsp_io_expander_init();
    ESP_RETURN_ON_FALSE(expander != NULL, ESP_FAIL, TAG, "Failed to initialize TCA9554");
    ESP_RETURN_ON_ERROR(
        esp_io_expander_set_dir(expander, BSP_LCD_RST | BSP_LCD_TOUCH_RST, IO_EXPANDER_OUTPUT), TAG, "");
    ESP_RETURN_ON_ERROR(
        esp_io_expander_set_level(expander, BSP_LCD_RST | BSP_LCD_TOUCH_RST, false), TAG, "");
    vTaskDelay(pdMS_TO_TICKS(BSP_TOUCH_RESET_DELAY_MS));
    ESP_RETURN_ON_ERROR(
        esp_io_expander_set_level(expander, BSP_LCD_RST | BSP_LCD_TOUCH_RST, true), TAG, "");
    vTaskDelay(pdMS_TO_TICKS(BSP_TOUCH_RESET_DELAY_MS));
    display_touch_reset_done = true;
    return ESP_OK;
}

bsp_board_variant_t bsp_board_detect(void)
{
    if (board_variant != BSP_BOARD_VARIANT_UNKNOWN) {
        return board_variant;
    }

    if (bsp_i2c_init() != ESP_OK || bsp_display_touch_reset() != ESP_OK) {
        return BSP_BOARD_VARIANT_UNKNOWN;
    }

    if (bsp_i2c_device_probe(BSP_TOUCH_CST820_I2C_ADDRESS) == ESP_OK) {
        board_variant = BSP_BOARD_VARIANT_CO5300_CST820;
    } else if (bsp_i2c_device_probe(BSP_TOUCH_FT5X06_I2C_ADDRESS) == ESP_OK) {
        board_variant = BSP_BOARD_VARIANT_SH8601_FT5X06;
    } else {
        ESP_LOGE(TAG, "No supported touch controller found at 0x%02X or 0x%02X",
                 BSP_TOUCH_CST820_I2C_ADDRESS, BSP_TOUCH_FT5X06_I2C_ADDRESS);
        return BSP_BOARD_VARIANT_UNKNOWN;
    }

    ESP_LOGI(TAG, "Detected board variant: %s", bsp_board_variant_to_name(board_variant));
    return board_variant;
}

bsp_board_variant_t bsp_board_get_variant(void)
{
    return board_variant;
}

const char *bsp_board_variant_to_name(bsp_board_variant_t variant)
{
    switch (variant) {
    case BSP_BOARD_VARIANT_SH8601_FT5X06:
        return "V1 (SH8601 + FT5x06)";
    case BSP_BOARD_VARIANT_CO5300_CST820:
        return "V2 (CO5300 + CST820)";
    default:
        return "unknown";
    }
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_sdcard_mount(void)
{
    if (bsp_sdcard != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lcd_spi_initialized || panel_handle != NULL) {
        ESP_LOGE(TAG, "Display and SD card cannot share SPI2 at the same time");
        return ESP_ERR_INVALID_STATE;
    }

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    const sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BSP_SD_SDCS;
    slot_config.host_id = host.slot;

    const spi_bus_config_t bus_cfg = {
        .mosi_io_num = BSP_SD_MOSI,
        .miso_io_num = BSP_SD_MISO,
        .sclk_io_num = BSP_SD_SCK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        return ret;
    }

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif

    ret = esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
    if (ret != ESP_OK) {
        bsp_sdcard = NULL;
        (void)spi_bus_free(host.slot);
    }
    return ret;
}

esp_err_t bsp_sdcard_unmount(void)
{
    if (bsp_sdcard == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    if (ret != ESP_OK) {
        return ret;
    }
    bsp_sdcard = NULL;
    return spi_bus_free(BSP_LCD_SPI_NUM);
}


esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_data_if != NULL) {
        /* Audio was initialized before */
        return ESP_OK;
    }
    if (i2s_tx_chan != NULL) {
        (void)i2s_channel_disable(i2s_tx_chan);
        (void)i2s_del_channel(i2s_tx_chan);
        i2s_tx_chan = NULL;
    }
    if (i2s_rx_chan != NULL) {
        (void)i2s_channel_disable(i2s_rx_chan);
        (void)i2s_del_channel(i2s_rx_chan);
        i2s_rx_chan = NULL;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (i2s_tx_chan != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(i2s_tx_chan));
    }

    if (i2s_rx_chan != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(i2s_rx_chan));
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_poweramp_enable(true));

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = BSP_I2S_NUM,
        .tx_handle = i2s_tx_chan,
        .rx_handle = i2s_rx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(i2s_data_if != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create audio data interface");

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_data_if == NULL) {
        if (bsp_i2c_init() != ESP_OK || bsp_audio_init(NULL) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize speaker audio interface");
            return NULL;
        }
    }
    ESP_RETURN_ON_FALSE(i2s_data_if != NULL, NULL, TAG, "Audio data interface is unavailable");

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if != NULL, NULL, TAG, "Failed to create audio GPIO interface");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(i2c_ctrl_if != NULL, NULL, TAG, "Failed to create audio control interface");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_TYPE_OUT,
        .pa_pin = GPIO_NUM_NC,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_dev != NULL, NULL, TAG, "Failed to create ES8311 speaker codec");

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8311_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_data_if == NULL) {
        if (bsp_i2c_init() != ESP_OK || bsp_audio_init(NULL) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize microphone audio interface");
            return NULL;
        }
    }
    ESP_RETURN_ON_FALSE(i2s_data_if != NULL, NULL, TAG, "Audio data interface is unavailable");

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if != NULL, NULL, TAG, "Failed to create audio GPIO interface");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(i2c_ctrl_if != NULL, NULL, TAG, "Failed to create audio control interface");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = GPIO_NUM_NC,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };

    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(es8311_dev != NULL, NULL, TAG, "Failed to create ES8311 microphone codec");

    esp_codec_dev_cfg_t codec_es8311_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es8311_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_es8311_dev_cfg);
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    ESP_RETURN_ON_FALSE(bsp_io_expander_init() != NULL, ESP_FAIL, TAG,
                        "Failed to initialize TCA9554");
    BSP_ERROR_CHECK_RETURN_ERR(
        esp_io_expander_set_dir(io_expander, BSP_POWER_AMP_IO, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_POWER_AMP_IO, (uint8_t)enable));

    return ESP_OK;
}

#define LCD_CMD_BITS (8)
#define LCD_PARAM_BITS (8)
esp_err_t bsp_display_brightness_init(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (panel_handle == NULL)
    {
        ESP_LOGE(TAG, "Panel handle is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (brightness_percent < 0 || brightness_percent > 100)
    {
        ESP_LOGE(TAG, "Invalid brightness percentage. Should be between 0 and 100.");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t lcd_cmd = 0x51;
    lcd_cmd &= 0xff;
    lcd_cmd <<= 8;
    lcd_cmd |= 0x02 << 24;
    uint8_t param = (uint8_t)(brightness_percent * 255 / 100);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io_handle, lcd_cmd, &param, 1), TAG, "");

    brightness = param;
    return ESP_OK;
}

int bsp_display_brightness_get(void)
{
    if (panel_handle == NULL)
    {
        ESP_LOGE(TAG, "Panel handle is not initialized");
        return -1;
    }

    return brightness * 100 / 255;
}

esp_err_t bsp_display_backlight_off(void)
{
    ESP_LOGI(TAG, "Backlight off");
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    ESP_LOGI(TAG, "Backlight on");
    return bsp_display_brightness_set(100);
}

#if LVGL_VERSION_MAJOR >= 9
static void rounder_event_cb(lv_event_t *e)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}
#else
static void bsp_lvgl_rounder_cb(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    (void)disp_drv;
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}
#endif

static esp_err_t bsp_display_create_sh8601(void)
{
    const esp_lcd_panel_io_spi_config_t io_config =
        SH8601_PANEL_IO_QSPI_CONFIG(BSP_LCD_CS, NULL, NULL);
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle),
        TAG, "");

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = sh8601_lcd_init_cmds,
        .init_cmds_size = sizeof(sh8601_lcd_init_cmds) / sizeof(sh8601_lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    return esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle);
}

static esp_err_t bsp_display_create_co5300(void)
{
    const esp_lcd_panel_io_spi_config_t io_config =
        CO5300_PANEL_IO_QSPI_CONFIG(BSP_LCD_CS, NULL, NULL);
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle),
        TAG, "");

    co5300_vendor_config_t vendor_config = {
        .init_cmds = co5300_lcd_init_cmds,
        .init_cmds_size = sizeof(co5300_lcd_init_cmds) / sizeof(co5300_lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(io_handle, &panel_config, &panel_handle), TAG, "");
    return esp_lcd_panel_set_gap(panel_handle, BSP_LCD_CO5300_X_GAP, 0);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    ESP_RETURN_ON_FALSE(bsp_sdcard == NULL, ESP_ERR_INVALID_STATE, TAG,
                        "Display and SD card cannot share SPI2 at the same time");
    ESP_RETURN_ON_FALSE(!lcd_spi_initialized && panel_handle == NULL && io_handle == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "Display is already initialized");

    const bsp_board_variant_t variant = bsp_board_detect();
    ESP_RETURN_ON_FALSE(variant != BSP_BOARD_VARIANT_UNKNOWN, ESP_ERR_NOT_FOUND, TAG,
                        "Unable to detect board variant");

    const size_t default_transfer_size =
        BSP_LCD_H_RES * BSP_LCD_V_RES * BSP_LCD_BITS_PER_PIXEL / 8;
    const size_t transfer_size =
        (config != NULL && config->max_transfer_sz > 0) ? config->max_transfer_sz : default_transfer_size;
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialize QSPI display for %s", bsp_board_variant_to_name(variant));
    if (variant == BSP_BOARD_VARIANT_CO5300_CST820) {
        const spi_bus_config_t bus_config =
            CO5300_PANEL_BUS_QSPI_CONFIG(BSP_LCD_PCLK, BSP_LCD_DATA0, BSP_LCD_DATA1,
                                         BSP_LCD_DATA2, BSP_LCD_DATA3, transfer_size);
        ret = spi_bus_initialize(BSP_LCD_SPI_NUM, &bus_config, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            return ret;
        }
        lcd_spi_initialized = true;
        ret = bsp_display_create_co5300();
    } else {
        const spi_bus_config_t bus_config =
            SH8601_PANEL_BUS_QSPI_CONFIG(BSP_LCD_PCLK, BSP_LCD_DATA0, BSP_LCD_DATA1,
                                         BSP_LCD_DATA2, BSP_LCD_DATA3, transfer_size);
        ret = spi_bus_initialize(BSP_LCD_SPI_NUM, &bus_config, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            return ret;
        }
        lcd_spi_initialized = true;
        ret = bsp_display_create_sh8601();
    }
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        goto fail;
    }
    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        goto fail;
    }
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK) {
        goto fail;
    }

    if (ret_panel != NULL) {
        *ret_panel = panel_handle;
    }
    if (ret_io != NULL) {
        *ret_io = io_handle;
    }
    return ESP_OK;

fail:
    if (panel_handle != NULL) {
        (void)esp_lcd_panel_del(panel_handle);
        panel_handle = NULL;
    }
    if (io_handle != NULL) {
        (void)esp_lcd_panel_io_del(io_handle);
        io_handle = NULL;
    }
    if (lcd_spi_initialized) {
        (void)spi_bus_free(BSP_LCD_SPI_NUM);
        lcd_spi_initialized = false;
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    (void)config;
    ESP_RETURN_ON_FALSE(ret_touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch output handle is required");

    const bsp_board_variant_t variant = bsp_board_detect();
    ESP_RETURN_ON_FALSE(variant != BSP_BOARD_VARIANT_UNKNOWN, ESP_ERR_NOT_FOUND, TAG,
                        "Unable to detect board variant");

    const esp_lcd_touch_config_t touch_config = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    esp_lcd_panel_io_i2c_config_t io_config;
    esp_err_t (*touch_new)(esp_lcd_panel_io_handle_t, const esp_lcd_touch_config_t *,
                           esp_lcd_touch_handle_t *) = NULL;
    if (variant == BSP_BOARD_VARIANT_CO5300_CST820) {
        const esp_lcd_panel_io_i2c_config_t cst816s_driver_config =
            ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
        io_config = cst816s_driver_config;
        touch_new = esp_lcd_touch_new_i2c_cst816s;
    } else {
        const esp_lcd_panel_io_i2c_config_t ft5x06_config =
            ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        io_config = ft5x06_config;
        touch_new = esp_lcd_touch_new_i2c_ft5x06;
    }

    io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;
    esp_lcd_panel_io_handle_t touch_io = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &io_config, &touch_io), TAG, "");
    return touch_new(touch_io, &touch_config, ret_touch);
}

/**************************************************************************************************
 *
 * IO Expander Function
 *
 **************************************************************************************************/
esp_io_expander_handle_t bsp_io_expander_init(void)
{
    if (bsp_i2c_init() != ESP_OK) {
        return NULL;
    }
    if (io_expander == NULL) {
        const esp_err_t ret =
            esp_io_expander_new_i2c_tca9554(i2c_handle, BSP_IO_EXPANDER_I2C_ADDRESS, &io_expander);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize TCA9554: %s", esp_err_to_name(ret));
            return NULL;
        }
    }
    return io_expander;
}

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    const bsp_display_config_t display_config = {
        .max_transfer_sz = cfg->trans_size,
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&display_config, &panel_handle, &io_handle));

    const lvgl_port_display_cfg_t display_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .monochrome = false,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .sw_rotate = true,
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
            .full_refresh = 0,
            .direct_mode = 0,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        },
    };

    lv_display_t *display = lvgl_port_add_disp(&display_cfg);
    if (display == NULL) {
        return NULL;
    }
#if LVGL_VERSION_MAJOR >= 9
    lv_display_add_event_cb(display, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
#else
    lv_disp_t *display_v8 = (lv_disp_t *)display;
    if (display_v8->driver != NULL) {
        display_v8->driver->rounder_cb = bsp_lvgl_rounder_cb;
    }
#endif
    return display;
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

/**********************************************************************************************************
 *
 * Display Function
 *
 **********************************************************************************************************/
lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = false,
        }};
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
