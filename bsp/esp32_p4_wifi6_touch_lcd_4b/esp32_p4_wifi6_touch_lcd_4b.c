#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_lcd_st7703.h"
#include "bsp/esp32_p4_wifi6_touch_lcd_4b.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch_gt911.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "ESP32_P4_4B";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool i2c_initialized = false;
static TaskHandle_t usb_host_task;  // USB Host Library task
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static i2c_master_bus_handle_t i2c_handle = NULL;  // I2C Handle
#endif
typedef enum {
    BSP_AUDIO_MODE_NONE,
    BSP_AUDIO_MODE_STD_STD,
    BSP_AUDIO_MODE_TX_STD_RX_TDM,
} bsp_audio_mode_t;

static bsp_audio_mode_t audio_mode = BSP_AUDIO_MODE_NONE;
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */

static esp_codec_dev_handle_t speaker_codec_dev = NULL;
static const audio_codec_gpio_if_t *speaker_gpio_if = NULL;
static const audio_codec_ctrl_if_t *speaker_ctrl_if = NULL;
static const audio_codec_if_t *speaker_codec_if = NULL;

static esp_codec_dev_handle_t microphone_codec_dev = NULL;
static const audio_codec_ctrl_if_t *microphone_ctrl_if = NULL;
static const audio_codec_if_t *microphone_codec_if = NULL;

#define BSP_ES7210_CODEC_ADDR  ES7210_CODEC_DEFAULT_ADDR

/* Can be used for I2S STD and TDM GPIO configuration */
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

/* This configuration is used by default in bsp_audio_init() */
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    return i2c_handle;
}

static esp_err_t bsp_i2c_device_probe(uint8_t address)
{
    return i2c_master_probe(i2c_handle, address, 100);
}

static esp_err_t bsp_enable_ldo_vo4(void)
{
    static esp_ldo_channel_handle_t vo4_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 4,
        .voltage_mv = 3300,
    };

    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &vo4_chan), TAG, "Acquire LDO VO4 channel failed");
    ESP_LOGI(TAG, "LDO VO4 set to 3300mV");

    return ESP_OK;
}

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 64 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;


    ESP_RETURN_ON_ERROR(bsp_enable_ldo_vo4(), TAG, "DSI PHY power failed");

    const sdmmc_slot_config_t slot_config = {
        /* SD card is connected to Slot 0 pins. Slot 0 uses IO MUX, so not specifying the pins here */
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };



    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
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
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

/**************************************************************************************************
 *
 * I2S Audio Function
 *
 **************************************************************************************************/
static void bsp_audio_update_result(esp_err_t *result, esp_err_t error)
{
    if ((*result == ESP_OK) && (error != ESP_OK)) {
        *result = error;
    }
}

static void bsp_audio_update_codec_result(esp_err_t *result, int codec_error)
{
    if ((*result == ESP_OK) && (codec_error != ESP_CODEC_DEV_OK)) {
        *result = ESP_FAIL;
    }
}

static bool bsp_audio_has_resources(void)
{
    return (audio_mode != BSP_AUDIO_MODE_NONE) ||
           (i2s_tx_chan != NULL) ||
           (i2s_rx_chan != NULL) ||
           (i2s_data_if != NULL) ||
           (speaker_codec_dev != NULL) ||
           (speaker_gpio_if != NULL) ||
           (speaker_ctrl_if != NULL) ||
           (speaker_codec_if != NULL) ||
           (microphone_codec_dev != NULL) ||
           (microphone_ctrl_if != NULL) ||
           (microphone_codec_if != NULL);
}

static esp_err_t bsp_audio_delete_speaker_codec(void)
{
    esp_err_t ret = ESP_OK;

    if (speaker_codec_dev != NULL) {
        bsp_audio_update_codec_result(&ret, esp_codec_dev_close(speaker_codec_dev));
        esp_codec_dev_delete(speaker_codec_dev);
        speaker_codec_dev = NULL;
    }
    if (speaker_codec_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_codec_if(speaker_codec_if));
        speaker_codec_if = NULL;
    }
    if (speaker_ctrl_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_ctrl_if(speaker_ctrl_if));
        speaker_ctrl_if = NULL;
    }
    if (speaker_gpio_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_gpio_if(speaker_gpio_if));
        speaker_gpio_if = NULL;
    }

    return ret;
}

static esp_err_t bsp_audio_delete_microphone_codec(void)
{
    esp_err_t ret = ESP_OK;

    if (microphone_codec_dev != NULL) {
        bsp_audio_update_codec_result(&ret, esp_codec_dev_close(microphone_codec_dev));
        esp_codec_dev_delete(microphone_codec_dev);
        microphone_codec_dev = NULL;
    }
    if (microphone_codec_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_codec_if(microphone_codec_if));
        microphone_codec_if = NULL;
    }
    if (microphone_ctrl_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_ctrl_if(microphone_ctrl_if));
        microphone_ctrl_if = NULL;
    }

    return ret;
}

static esp_err_t bsp_audio_delete_i2s_channel(i2s_chan_handle_t *channel)
{
    esp_err_t ret = ESP_OK;
    esp_err_t err;

    if (*channel == NULL) {
        return ESP_OK;
    }

    err = i2s_channel_disable(*channel);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ret = err;
    }

    err = i2s_del_channel(*channel);
    bsp_audio_update_result(&ret, err);
    *channel = NULL;

    return ret;
}

esp_err_t bsp_audio_deinit(void)
{
    esp_err_t ret = ESP_OK;

    bsp_audio_update_result(&ret, bsp_audio_delete_microphone_codec());
    bsp_audio_update_result(&ret, bsp_audio_delete_speaker_codec());

    if (i2s_data_if != NULL) {
        bsp_audio_update_codec_result(&ret, audio_codec_delete_data_if(i2s_data_if));
        i2s_data_if = NULL;
    }

    bsp_audio_update_result(&ret, bsp_audio_delete_i2s_channel(&i2s_rx_chan));
    bsp_audio_update_result(&ret, bsp_audio_delete_i2s_channel(&i2s_tx_chan));
    audio_mode = BSP_AUDIO_MODE_NONE;

    return ret;
}

static esp_err_t bsp_audio_init_channels(const i2s_std_config_t *tx_config,
                                         const i2s_std_config_t *rx_std_config,
                                         const i2s_tdm_config_t *rx_tdm_config,
                                         bsp_audio_mode_t mode)
{
    esp_err_t ret = ESP_OK;
    esp_err_t cleanup_ret;

    if (bsp_audio_has_resources()) {
        return ESP_ERR_INVALID_STATE;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    ESP_GOTO_ON_ERROR(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan),
                      err, TAG, "I2S channel creation failed");
    ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_tx_chan, tx_config),
                      err, TAG, "I2S TX STD initialization failed");

    if (rx_tdm_config != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_tdm_mode(i2s_rx_chan, rx_tdm_config),
                          err, TAG, "I2S RX TDM initialization failed");
    } else {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_rx_chan, rx_std_config),
                          err, TAG, "I2S RX STD initialization failed");
    }

    ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, TAG, "I2S TX enabling failed");
    ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "I2S RX enabling failed");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (i2s_data_if == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto err;
    }

    audio_mode = mode;
    return ESP_OK;

err:
    cleanup_ret = bsp_audio_deinit();
    if (cleanup_ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio cleanup failed: %s", esp_err_to_name(cleanup_ret));
    }
    return ret;
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if ((audio_mode == BSP_AUDIO_MODE_STD_STD) &&
            (i2s_tx_chan != NULL) && (i2s_rx_chan != NULL) && (i2s_data_if != NULL)) {
        return ESP_OK;
    }
    if (bsp_audio_has_resources()) {
        return ESP_ERR_INVALID_STATE;
    }

    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *std_config = (i2s_config != NULL) ? i2s_config : &std_cfg_default;

    return bsp_audio_init_channels(std_config, std_config, NULL, BSP_AUDIO_MODE_STD_STD);
}

esp_err_t bsp_audio_init_tx_std_rx_tdm(const i2s_std_config_t *tx_config,
                                       const i2s_tdm_config_t *rx_config)
{
    if (bsp_audio_has_resources()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (tx_config == NULL || rx_config == NULL ||
            tx_config->clk_cfg.sample_rate_hz != rx_config->clk_cfg.sample_rate_hz) {
        return ESP_ERR_INVALID_ARG;
    }

    return bsp_audio_init_channels(tx_config, NULL, rx_config, BSP_AUDIO_MODE_TX_STD_RX_TDM);
}

esp_err_t bsp_audio_init_voice_24k(void)
{
    i2s_std_config_t tx_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    i2s_tdm_config_t rx_config = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg = I2S_TDM_PHILIP_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO,
            I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    tx_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    tx_config.gpio_cfg.din = I2S_GPIO_UNUSED;
    rx_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    rx_config.clk_cfg.bclk_div = 8;
    rx_config.slot_cfg.total_slot = BSP_AUDIO_TDM_SLOT_COUNT;
    rx_config.gpio_cfg.dout = I2S_GPIO_UNUSED;

    return bsp_audio_init_tx_std_rx_tdm(&tx_config, &rx_config);
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (speaker_codec_dev != NULL) {
        return speaker_codec_dev;
    }

    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
    if (i2s_data_if == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
    }

    speaker_gpio_if = audio_codec_new_gpio();
    if (speaker_gpio_if == NULL) {
        goto err;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    speaker_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (speaker_ctrl_if == NULL) {
        goto err;
    }

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = speaker_ctrl_if,
        .gpio_if = speaker_gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    speaker_codec_if = es8311_codec_new(&es8311_cfg);
    if (speaker_codec_if == NULL) {
        goto err;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = speaker_codec_if,
        .data_if = i2s_data_if,
    };
    speaker_codec_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (speaker_codec_dev == NULL) {
        goto err;
    }

    return speaker_codec_dev;

err:
    bsp_audio_delete_speaker_codec();
    return NULL;
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (microphone_codec_dev != NULL) {
        return microphone_codec_dev;
    }

    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
    if (i2s_data_if == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES7210_CODEC_ADDR,
        .bus_handle = i2c_handle,
    };
    microphone_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (microphone_ctrl_if == NULL) {
        goto err;
    }

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = microphone_ctrl_if,
        .mic_selected = (audio_mode == BSP_AUDIO_MODE_TX_STD_RX_TDM) ?
                        BSP_AUDIO_ES7210_CONNECTED_MIC_MASK : 0,
    };
    microphone_codec_if = es7210_codec_new(&es7210_cfg);
    if (microphone_codec_if == NULL) {
        goto err;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = microphone_codec_if,
        .data_if = i2s_data_if,
    };
    microphone_codec_dev = esp_codec_dev_new(&codec_dev_cfg);
    if (microphone_codec_dev == NULL) {
        goto err;
    }

    return microphone_codec_dev;

err:
    bsp_audio_delete_microphone_codec();
    return NULL;
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0,
        .flags = { .output_invert = 1 }
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    } else if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);

    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif // BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    bsp_lcd_handles_t handles;
    ret = bsp_display_new_with_handles(config, &handles);

    *ret_panel = handles.panel;
    *ret_io = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");
    ESP_RETURN_ON_ERROR(bsp_enable_ldo_vo4(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = 0,
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD ILI9881C spec
        .lcd_param_bits = 8, // according to the LCD ILI9881C spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    esp_lcd_panel_handle_t disp_panel = NULL;
    ESP_LOGI(TAG, "Install Waveshare ESP32-P4-WIFI6-Touch-LCD-4B LCD control panel");
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = ST7703_720_720_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = ST7703_720_720_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    st7703_vendor_config_t vendor_config = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .bits_per_pixel = 24,
#else
        .bits_per_pixel = 16,
#endif
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = BSP_LCD_RST,
        .vendor_config = &vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7703(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel Waveshare failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");

    /* Return all handles */
    ret_handles->io = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = disp_panel;
    ret_handles->control = NULL;

    ESP_LOGI(TAG, "Display initialized");

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    const bsp_touch_config_t default_config = {
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    if (config == NULL) {
        config = &default_config;
    }

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
#if CONFIG_BSP_LCD_TYPE_480_640_2_8_INCH || CONFIG_BSP_LCD_TYPE_480_800_4_INCH
        .x_max = BSP_LCD_V_RES,
        .y_max = BSP_LCD_H_RES,
#else
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
#endif
        .rst_gpio_num = BSP_LCD_TOUCH_RST,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = config->flags.swap_xy,
            .mirror_x = config->flags.mirror_x,
            .mirror_y = config->flags.mirror_y,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    if (bsp_i2c_device_probe(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS) == ESP_OK) {
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS;
    } else if (bsp_i2c_device_probe(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP) == ESP_OK) {
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
    } else {
        ESP_LOGE(TAG, "GT911 not found at 0x%02X or 0x%02X",
                 ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
                 ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "GT911 found at 0x%02X", tp_io_config.dev_addr);
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = lcd_panels.panel,
        .panel_io = lcd_panels.io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_MIPI_DSI,
            .rotation = cfg->rotation,
            .hor_res = BSP_LCD_H_RES,
            .ver_res = BSP_LCD_V_RES,
            .buffer_height = 50,
            .use_psram = false,
            .enable_ppa_accel = false,
            .require_double_buffer = false,
        },
        .tear_avoid_mode = cfg->tear_avoid_mode,
    };

    return esp_lv_adapter_register_display(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(const bsp_display_cfg_t *cfg, lv_display_t *disp)
{
    assert(cfg != NULL);
    esp_lcd_touch_handle_t tp;
    const bsp_touch_config_t touch_config = {
        .flags = {
            .swap_xy = cfg->touch_flags.swap_xy,
            .mirror_x = cfg->touch_flags.mirror_x,
            .mirror_y = cfg->touch_flags.mirror_y,
        },
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(&touch_config, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const esp_lv_adapter_touch_config_t touch_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, tp);

    return esp_lv_adapter_register_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        .touch_flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_init(&cfg->lv_adapter_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(cfg, disp), NULL);

    ESP_ERROR_CHECK(esp_lv_adapter_start());

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

bool bsp_display_lock(int32_t timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

void bsp_display_unlock(void)
{
    esp_lv_adapter_unlock();
}

#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
        }
    }
}

esp_err_t bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA)
{
    //Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    BSP_ERROR_CHECK_RETURN_ERR(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, &usb_host_task) != pdTRUE) {
        ESP_LOGE(TAG, "Creating USB host lib task failed");
        abort();
    }

    return ESP_OK;
}

esp_err_t bsp_usb_host_stop(void)
{
    usb_host_uninstall();
    if (usb_host_task) {
        vTaskSuspend(usb_host_task);
        vTaskDelete(usb_host_task);
    }
    return ESP_OK;
}
