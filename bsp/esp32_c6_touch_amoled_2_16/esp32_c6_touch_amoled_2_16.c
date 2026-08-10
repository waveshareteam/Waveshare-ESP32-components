/*
 * SPDX-FileCopyrightText: 2026 Waveshare Team
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_codec_dev_defaults.h"
#include "esp_lcd_sh8601.h"
#include "esp_lcd_touch_cst9217.h"

#include "bsp/esp32_c6_touch_amoled_2_16.h"
#include "bsp_err_check.h"

static const char *TAG = "ESP32-C6-Touch-AMOLED-2.16";

static i2c_master_bus_handle_t i2c_handle;
static i2c_master_dev_handle_t pmu_handle;
static SemaphoreHandle_t pmu_mutex;
static bool i2c_initialized;
static bool pmu_initialized;
static bool spi2_initialized;

static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;
static const audio_codec_data_if_t *i2s_data_if;
static esp_codec_dev_handle_t speaker_codec;
static esp_codec_dev_handle_t microphone_codec;

static esp_lcd_panel_handle_t panel_handle;
static esp_lcd_panel_io_handle_t panel_io_handle;
static esp_lcd_touch_handle_t touch_handle;
#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *display_indev;
#endif
static uint8_t display_brightness;

sdmmc_card_t *bsp_sdcard;

/* AXP2101 registers used by this board. */
#define AXP2101_REG_STATUS1          (0x00)
#define AXP2101_REG_STATUS2          (0x01)
#define AXP2101_REG_CHIP_ID          (0x03)
#define AXP2101_REG_INPUT_CURRENT    (0x16)
#define AXP2101_REG_ADC_CHANNEL      (0x30)
#define AXP2101_REG_BAT_VOLTAGE_H    (0x34)
#define AXP2101_REG_BAT_VOLTAGE_L    (0x35)
#define AXP2101_REG_PRECHARGE        (0x61)
#define AXP2101_REG_CHARGE_CURRENT   (0x62)
#define AXP2101_REG_TERMINATION      (0x63)
#define AXP2101_REG_BAT_DETECTION    (0x68)
#define AXP2101_REG_DCDC_ONOFF       (0x80)
#define AXP2101_REG_DCDC1_VOLTAGE    (0x82)
#define AXP2101_REG_LDO_ONOFF        (0x90)
#define AXP2101_REG_ALDO1_VOLTAGE    (0x92)
#define AXP2101_REG_ALDO2_VOLTAGE    (0x93)
#define AXP2101_REG_ALDO3_VOLTAGE    (0x94)
#define AXP2101_REG_ALDO4_VOLTAGE    (0x95)
#define AXP2101_CHIP_ID              (0x4A)

#define AXP2101_VBUS_LIMIT_2000MA    (5U)
#define AXP2101_PRECHARGE_50MA       (2U)
#define AXP2101_CHARGE_500MA         (11U)
#define AXP2101_TERMINATION_50MA     (2U)

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

#define BSP_I2S_DUPLEX_STEREO_CFG(_sample_rate)                                                   \
    {                                                                                              \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                       \
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                              \
    }

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 600},
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x19, (uint8_t[]){0x10}, 1, 0},
    {0x1C, (uint8_t[]){0xA0}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0x30}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4, 0},
    {0x29, (uint8_t[]){0x00}, 0, 100},
};

static esp_err_t pmu_read_locked(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(pmu_handle, &reg, 1, value, 1, 100);
}

static esp_err_t pmu_write_locked(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(pmu_handle, data, sizeof(data), 100);
}

static esp_err_t pmu_update_locked(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    ESP_RETURN_ON_ERROR(pmu_read_locked(reg, &current), TAG, "Read AXP2101 register 0x%02X", reg);
    current = (current & (uint8_t)~mask) | (value & mask);
    return pmu_write_locked(reg, current);
}

static esp_err_t pmu_set_aldo_voltage_locked(uint8_t reg, uint16_t millivolts)
{
    ESP_RETURN_ON_FALSE(millivolts >= 500 && millivolts <= 3500 && millivolts % 100 == 0,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid ALDO voltage: %u mV", millivolts);
    return pmu_update_locked(reg, 0x1F, (uint8_t)((millivolts - 500) / 100));
}

static esp_err_t pmu_set_ldo_power(uint8_t bit, bool enable)
{
    ESP_RETURN_ON_ERROR(bsp_pmu_init(), TAG, "PMU is unavailable");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(pmu_mutex, pdMS_TO_TICKS(100)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "Timed out waiting for PMU");
    const esp_err_t ret = pmu_update_locked(AXP2101_REG_LDO_ONOFF, (uint8_t)(1U << bit),
                                            enable ? (uint8_t)(1U << bit) : 0);
    xSemaphoreGive(pmu_mutex);
    return ret;
}

esp_err_t bsp_i2c_init(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t config = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&config, &i2c_handle));
    i2c_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    if (!i2c_initialized) {
        return ESP_OK;
    }
    if (pmu_handle != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2c_master_bus_rm_device(pmu_handle));
        pmu_handle = NULL;
        pmu_initialized = false;
    }
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_handle = NULL;
    i2c_initialized = false;
    if (pmu_mutex != NULL) {
        vSemaphoreDelete(pmu_mutex);
        pmu_mutex = NULL;
    }
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    return bsp_i2c_init() == ESP_OK ? i2c_handle : NULL;
}

esp_err_t bsp_pmu_init(void)
{
    if (pmu_initialized) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "Failed to initialize I2C");

    if (pmu_mutex == NULL) {
        pmu_mutex = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(pmu_mutex != NULL, ESP_ERR_NO_MEM, TAG, "Failed to create PMU mutex");
    }
    if (pmu_handle == NULL) {
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BSP_PMU_I2C_ADDRESS,
            .scl_speed_hz = 100000,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_handle, &config, &pmu_handle),
                            TAG, "Failed to register AXP2101");
    }

    ESP_RETURN_ON_FALSE(xSemaphoreTake(pmu_mutex, pdMS_TO_TICKS(100)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "Timed out waiting for PMU");
    esp_err_t ret;
    uint8_t chip_id;
    ret = pmu_read_locked(AXP2101_REG_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        goto out;
    }
    if (chip_id != AXP2101_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected PMU chip ID: 0x%02X", chip_id);
        ret = ESP_ERR_NOT_FOUND;
        goto out;
    }

    /* Values are taken from the product schematic and factory examples. */
    if ((ret = pmu_update_locked(AXP2101_REG_INPUT_CURRENT, 0x07, AXP2101_VBUS_LIMIT_2000MA)) != ESP_OK ||
        (ret = pmu_write_locked(AXP2101_REG_DCDC1_VOLTAGE, (3300 - 1500) / 100)) != ESP_OK ||
        (ret = pmu_set_aldo_voltage_locked(AXP2101_REG_ALDO1_VOLTAGE, 3300)) != ESP_OK ||
        (ret = pmu_set_aldo_voltage_locked(AXP2101_REG_ALDO2_VOLTAGE, 3300)) != ESP_OK ||
        (ret = pmu_set_aldo_voltage_locked(AXP2101_REG_ALDO3_VOLTAGE, 3300)) != ESP_OK ||
        (ret = pmu_set_aldo_voltage_locked(AXP2101_REG_ALDO4_VOLTAGE, 1800)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_DCDC_ONOFF, 0x01, 0x01)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_LDO_ONOFF, 0x09, 0x09)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_PRECHARGE, 0x03, AXP2101_PRECHARGE_50MA)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_CHARGE_CURRENT, 0x1F, AXP2101_CHARGE_500MA)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_TERMINATION, 0x0F, AXP2101_TERMINATION_50MA)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_ADC_CHANNEL, 0x03, 0x01)) != ESP_OK ||
        (ret = pmu_update_locked(AXP2101_REG_BAT_DETECTION, 0x01, 0x01)) != ESP_OK) {
        goto out;
    }
    pmu_initialized = true;
    ESP_LOGI(TAG, "AXP2101 initialized");

out:
    xSemaphoreGive(pmu_mutex);
    return ret;
}

esp_err_t bsp_pmu_get_snapshot(bsp_pmu_snapshot_t *snapshot)
{
    ESP_RETURN_ON_FALSE(snapshot != NULL, ESP_ERR_INVALID_ARG, TAG, "Snapshot is required");
    ESP_RETURN_ON_ERROR(bsp_pmu_init(), TAG, "PMU is unavailable");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(pmu_mutex, pdMS_TO_TICKS(100)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "Timed out waiting for PMU");

    uint8_t status1;
    uint8_t status2;
    uint8_t voltage_h;
    uint8_t voltage_l;
    esp_err_t ret = pmu_read_locked(AXP2101_REG_STATUS1, &status1);
    if (ret == ESP_OK) {
        ret = pmu_read_locked(AXP2101_REG_STATUS2, &status2);
    }
    if (ret == ESP_OK) {
        ret = pmu_read_locked(AXP2101_REG_BAT_VOLTAGE_H, &voltage_h);
    }
    if (ret == ESP_OK) {
        ret = pmu_read_locked(AXP2101_REG_BAT_VOLTAGE_L, &voltage_l);
    }
    if (ret == ESP_OK) {
        snapshot->battery_present = (status1 & (1U << 3)) != 0;
        snapshot->charging = ((status2 >> 5) & 0x03) == 1;
        snapshot->charger_status = (bsp_pmu_charger_status_t)(status2 & 0x07);
        snapshot->battery_mv = (uint16_t)(((voltage_h & 0x1F) << 8) | voltage_l);
    }
    xSemaphoreGive(pmu_mutex);
    return ret;
}

const char *bsp_pmu_charger_status_to_string(bsp_pmu_charger_status_t status)
{
    switch (status) {
    case BSP_PMU_CHARGE_TRI_STATE:
        return "tri-state";
    case BSP_PMU_CHARGE_PRECHARGE:
        return "pre-charge";
    case BSP_PMU_CHARGE_CONSTANT_CURRENT:
        return "constant-current";
    case BSP_PMU_CHARGE_CONSTANT_VOLTAGE:
        return "constant-voltage";
    case BSP_PMU_CHARGE_DONE:
        return "done";
    case BSP_PMU_CHARGE_STOPPED:
        return "stopped";
    default:
        return "unknown";
    }
}

esp_err_t bsp_pmu_set_display_power(bool enable)
{
    return pmu_set_ldo_power(2, enable);
}

esp_err_t bsp_pmu_set_audio_power(bool enable)
{
    return pmu_set_ldo_power(1, enable);
}

static esp_err_t bsp_spi2_init(void)
{
    if (spi2_initialized) {
        return ESP_OK;
    }
    const int max_transfer_size = BSP_LCD_H_RES * BSP_LCD_V_RES * BSP_LCD_BITS_PER_PIXEL / 8;
    const spi_bus_config_t bus_config = SH8601_PANEL_BUS_QSPI_CONFIG(
        BSP_LCD_PCLK, BSP_LCD_DATA0, BSP_LCD_DATA1, BSP_LCD_DATA2, BSP_LCD_DATA3,
        max_transfer_size);
    BSP_ERROR_CHECK_RETURN_ERR(spi_bus_initialize(BSP_LCD_SPI_NUM, &bus_config, SPI_DMA_CH_AUTO));
    spi2_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    const esp_vfs_spiffs_conf_t config = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };
    ESP_RETURN_ON_ERROR(esp_vfs_spiffs_register(&config), TAG, "Failed to mount SPIFFS");

    size_t total = 0;
    size_t used = 0;
    const esp_err_t ret = esp_spiffs_info(config.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: total=%zu used=%zu", total, used);
    }
    return ret;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_sdcard_mount(void)
{
    ESP_RETURN_ON_FALSE(bsp_sdcard == NULL, ESP_ERR_INVALID_STATE, TAG, "SD card is already mounted");
    ESP_RETURN_ON_ERROR(bsp_spi2_init(), TAG, "Failed to initialize shared SPI2 bus");

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = BSP_LCD_SPI_NUM;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = BSP_LCD_SPI_NUM;
    slot_config.gpio_cs = BSP_SD_SDCS;

    const esp_err_t ret = esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_config,
                                                   &mount_config, &bsp_sdcard);
    if (ret != ESP_OK) {
        bsp_sdcard = NULL;
    }
    return ret;
}

esp_err_t bsp_sdcard_unmount(void)
{
    ESP_RETURN_ON_FALSE(bsp_sdcard != NULL, ESP_ERR_INVALID_STATE, TAG, "SD card is not mounted");
    const esp_err_t ret = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    if (ret == ESP_OK) {
        bsp_sdcard = NULL;
    }
    return ret;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    return bsp_pmu_set_audio_power(enable);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_data_if != NULL) {
        return ESP_OK;
    }

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    channel_config.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, &i2s_tx_chan, &i2s_rx_chan),
                        TAG, "Failed to create I2S channels");

    const i2s_std_config_t default_config = BSP_I2S_DUPLEX_STEREO_CFG(16000);
    const i2s_std_config_t *config = i2s_config != NULL ? i2s_config : &default_config;
    esp_err_t ret = i2s_channel_init_std_mode(i2s_tx_chan, config);
    if (ret == ESP_OK) {
        ret = i2s_channel_init_std_mode(i2s_rx_chan, config);
    }
    if (ret == ESP_OK) {
        ret = i2s_channel_enable(i2s_tx_chan);
    }
    if (ret == ESP_OK) {
        ret = i2s_channel_enable(i2s_rx_chan);
    }
    if (ret != ESP_OK) {
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
        return ret;
    }

    audio_codec_i2s_cfg_t codec_i2s_config = {
        .port = BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&codec_i2s_config);
    return i2s_data_if != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (speaker_codec != NULL) {
        return speaker_codec;
    }
    if (bsp_i2c_init() != ESP_OK || bsp_audio_init(NULL) != ESP_OK ||
        bsp_audio_poweramp_enable(true) != ESP_OK) {
        return NULL;
    }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    BSP_NULL_CHECK(gpio_if, NULL);
    audio_codec_i2c_cfg_t i2c_config = {
        .port = BSP_I2C_NUM,
        .addr = BSP_AUDIO_OUT_I2C_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *control_if = audio_codec_new_i2c_ctrl(&i2c_config);
    BSP_NULL_CHECK(control_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };
    es8311_codec_cfg_t codec_config = {
        .ctrl_if = control_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = GPIO_NUM_NC,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&codec_config);
    BSP_NULL_CHECK(codec_if, NULL);
    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if = i2s_data_if,
    };
    speaker_codec = esp_codec_dev_new(&device_config);
    return speaker_codec;
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (microphone_codec != NULL) {
        return microphone_codec;
    }
    if (bsp_i2c_init() != ESP_OK || bsp_audio_init(NULL) != ESP_OK) {
        return NULL;
    }

    audio_codec_i2c_cfg_t i2c_config = {
        .port = BSP_I2C_NUM,
        .addr = BSP_AUDIO_IN_I2C_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *control_if = audio_codec_new_i2c_ctrl(&i2c_config);
    BSP_NULL_CHECK(control_if, NULL);
    es7210_codec_cfg_t codec_config = {
        .ctrl_if = control_if,
    };
    const audio_codec_if_t *codec_if = es7210_codec_new(&codec_config);
    BSP_NULL_CHECK(codec_if, NULL);
    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec_if,
        .data_if = i2s_data_if,
    };
    microphone_codec = esp_codec_dev_new(&device_config);
    return microphone_codec;
}

esp_err_t bsp_display_brightness_init(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    ESP_RETURN_ON_FALSE(panel_io_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Display is not initialized");
    ESP_RETURN_ON_FALSE(brightness_percent >= 0 && brightness_percent <= 100,
                        ESP_ERR_INVALID_ARG, TAG, "Brightness must be in the range 0..100");

    const uint32_t command = ((uint32_t)0x02 << 24) | ((uint32_t)0x51 << 8);
    const uint8_t value = (uint8_t)(brightness_percent * 255 / 100);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(panel_io_handle, command, &value, 1), TAG, "");
    display_brightness = value;
    return ESP_OK;
}

int bsp_display_brightness_get(void)
{
    return panel_io_handle != NULL ? display_brightness * 100 / 255 : -1;
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config,
                          esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    (void)config;
    ESP_RETURN_ON_FALSE(panel_handle == NULL && panel_io_handle == NULL,
                        ESP_ERR_INVALID_STATE, TAG, "Display is already initialized");
    ESP_RETURN_ON_ERROR(bsp_pmu_init(), TAG, "Failed to initialize PMU");
    ESP_RETURN_ON_ERROR(bsp_pmu_set_display_power(false), TAG, "Failed to reset display power");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(bsp_pmu_set_display_power(true), TAG, "Failed to enable display power");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(bsp_spi2_init(), TAG, "Failed to initialize shared SPI2 bus");

    const esp_lcd_panel_io_spi_config_t io_config =
        SH8601_PANEL_IO_QSPI_CONFIG(BSP_LCD_CS, NULL, NULL);
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config,
                                 &panel_io_handle),
        TAG, "Failed to create SH8601 panel IO");

    const sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    esp_err_t ret = esp_lcd_new_panel_sh8601(panel_io_handle, &panel_config, &panel_handle);
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_reset(panel_handle);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_init(panel_handle);
    }
    if (ret == ESP_OK) {
        ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    }
    if (ret != ESP_OK) {
        if (panel_handle != NULL) {
            (void)esp_lcd_panel_del(panel_handle);
            panel_handle = NULL;
        }
        if (panel_io_handle != NULL) {
            (void)esp_lcd_panel_io_del(panel_io_handle);
            panel_io_handle = NULL;
        }
        return ret;
    }

    if (ret_panel != NULL) {
        *ret_panel = panel_handle;
    }
    if (ret_io != NULL) {
        *ret_io = panel_io_handle;
    }
    return ESP_OK;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    ESP_RETURN_ON_FALSE(ret_touch != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch output is required");
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "Failed to initialize I2C");

    const bsp_touch_config_t default_config = {
        .swap_xy = true,
        .mirror_x = false,
        .mirror_y = true,
    };
    const bsp_touch_config_t *touch_config = config != NULL ? config : &default_config;
    const esp_lcd_touch_config_t driver_config = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = BSP_LCD_TOUCH_RST,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = touch_config->swap_xy,
            .mirror_x = touch_config->mirror_x,
            .mirror_y = touch_config->mirror_y,
        },
    };
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
    io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;
    esp_lcd_panel_io_handle_t touch_io = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &io_config, &touch_io), TAG, "");
    return esp_lcd_touch_new_i2c_cst9217(touch_io, &driver_config, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#if LVGL_VERSION_MAJOR >= 9
static void rounder_event_cb(lv_event_t *event)
{
    lv_area_t *area = (lv_area_t *)lv_event_get_param(event);
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}
#else
static void rounder_cb(lv_disp_drv_t *display_driver, lv_area_t *area)
{
    (void)display_driver;
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}
#endif

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *config)
{
    const bsp_display_config_t panel_config = {
        .max_transfer_sz = BSP_LCD_H_RES * BSP_LCD_V_RES * BSP_LCD_BITS_PER_PIXEL / 8,
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&panel_config, &panel_handle, &panel_io_handle));

    esp_lv_adapter_display_config_t display_config = {
        .panel = panel_handle,
        .panel_io = panel_io_handle,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = config->rotation,
            .hor_res = BSP_LCD_H_RES,
            .ver_res = BSP_LCD_V_RES,
            .buffer_height = CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT,
            .use_psram = false,
            .enable_ppa_accel = false,
            .require_double_buffer = false,
        },
        .tear_avoid_mode = config->tear_avoid_mode,
    };
    lv_display_t *display = esp_lv_adapter_register_display(&display_config);
    if (display == NULL) {
        return NULL;
    }
#if LVGL_VERSION_MAJOR >= 9
    lv_display_add_event_cb(display, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
#else
    lv_disp_t *display_v8 = (lv_disp_t *)display;
    if (display_v8->driver != NULL) {
        display_v8->driver->rounder_cb = rounder_cb;
    }
#endif
    return display;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t config = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
        .touch = {
            .swap_xy = true,
            .mirror_x = false,
            .mirror_y = true,
        },
    };
    return bsp_display_start_with_config(&config);
}

lv_display_t *bsp_display_start_with_config(bsp_display_cfg_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, NULL, TAG, "Display configuration is required");
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_init(&config->lv_adapter_cfg));

    lv_display_t *display = bsp_display_lcd_init(config);
    BSP_NULL_CHECK(display, NULL);
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(&config->touch, &touch_handle));

    esp_lv_adapter_touch_config_t touch_config =
        ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(display, touch_handle);
    display_indev = esp_lv_adapter_register_touch(&touch_config);
    BSP_NULL_CHECK(display_indev, NULL);
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_start());
    return display;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return display_indev;
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    esp_lv_adapter_unlock();
}
#endif
