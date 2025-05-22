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

#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH || CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A
#include "esp_lcd_jd9365_10_1.h"
#elif CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A
#include "esp_lcd_jd9365_8.h"
#elif CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A
#include "esp_lcd_ili9881c.h"
#else
#include "esp_lcd_dsi.h"
#endif

#include "bsp/esp32_p4_nano.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch_gt911.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "ESP32_P4_EV";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool i2c_initialized = false;
static TaskHandle_t usb_host_task;  // USB Host Library task
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static i2c_master_bus_handle_t i2c_handle = NULL;  // I2C Handle
#endif
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */

/* Can be used for `i2s_std_gpio_config_t` and/or `i2s_std_config_t` initialization */
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

/* This configuration is used by default in `bsp_extra_audio_init()` */
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

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;

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

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (i2s_tx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    }

    if (i2s_rx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .tx_handle = i2s_tx_chan,
        .rx_handle = i2s_rx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        ESP_ERROR_CHECK(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL));
    }
    assert(i2s_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_TYPE_OUT,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    assert(es8311_dev);

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
        /* Initilize I2C */
        ESP_ERROR_CHECK(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL));
    }
    assert(i2s_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };

    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    assert(es8311_dev);

    esp_codec_dev_cfg_t codec_es8311_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es8311_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_es8311_dev_cfg);
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    bsp_i2c_init();
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    uint8_t data = (uint8_t)(255 * brightness_percent * 0.01);
    uint8_t chip_addr = 0x45;
#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH
    uint8_t data_addr = 0x86;
    uint8_t data_to_send[2] = {data_addr, data};
#elif CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A || CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A || CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A
    uint8_t data_addr = 0x96;
    uint8_t data_to_send[2] = {data_addr, data};
#else
    uint8_t data_addr = 0xab;
    uint8_t data_to_send[2] = {data_addr, 0xff - data};
#endif

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 100 * 1000,
        .device_address = chip_addr,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    if (i2c_master_bus_add_device(i2c_handle, &i2c_dev_conf, &dev_handle) != ESP_OK)
    {
        return ESP_FAIL;
    }


    esp_err_t ret = i2c_master_transmit(dev_handle, data_to_send, sizeof(data_to_send), 50);
    if (ret != ESP_OK)
    {
        i2c_master_bus_rm_device(dev_handle);
        return ret;
    }

    i2c_master_bus_rm_device(dev_handle);

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

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
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
#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH || CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A
    ESP_LOGI(TAG, "Install Waveshare 10.1-DSI-TOUCH-A or 101M-8001280-IPS-CT-K LCD control panel");

#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    jd9365_vendor_config_t vendor_config = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = 2,
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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_jd9365(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel Waveshare failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");

#elif CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A
    ESP_LOGI(TAG, "Install Waveshare 8-DSI-TOUCH-A LCD control panel");

#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_8_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = JD9365_8_800_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    jd9365_8_vendor_config_t vendor_config = {
        .flags = {
            .use_mipi_interface = 1,
        },
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = 2,
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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_jd9365_8(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel Waveshare failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");
#elif CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A
    ESP_LOGI(TAG, "Install Waveshare 7-DSI-TOUCH-A LCD control panel");

#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
    esp_lcd_dpi_panel_config_t dpi_config = ILI9881C_720_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#else
    esp_lcd_dpi_panel_config_t dpi_config = ILI9881C_720_1280_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    ili9881c_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num = 2,
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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel ILI9881C failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(disp_panel, true), err, TAG, "LCD panel ON failed");
#else
    ESP_LOGI(TAG, "Install Waveshare DSI LCD control panel");
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
#if CONFIG_BSP_LCD_TYPE_480_640_2_8_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_2_8_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_800_800_3_4_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_3_4_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_720_720_4_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_4_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_480_800_4_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_4_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_720_1280_5_INCH_D
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_5_INCH_D_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_720_1560_6_25_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_6_25_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_1024_600_5_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_5_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_1024_600_7_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_400_1280_7_9_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_9_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_1280_800_7_INCH_E
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_INCH_E_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_1280_800_8_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_8_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_1280_800_10_1_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_10_1_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_480_1920_8_8_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_8_8_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#elif CONFIG_BSP_LCD_TYPE_320_1480_11_9_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_11_9_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB888);
#endif

#else
#if CONFIG_BSP_LCD_TYPE_480_640_2_8_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_2_8_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_800_800_3_4_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_3_4_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_720_720_4_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_4_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_480_800_4_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_4_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_720_1280_5_INCH_D
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_5_INCH_D_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_720_1560_6_25_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_6_25_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_1024_600_5_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_5_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_1024_600_7_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_400_1280_7_9_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_9_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_1280_800_7_INCH_E
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_7_INCH_E_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_1280_800_8_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_8_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_1280_800_10_1_INCH_C
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_10_1_INCH_C_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_480_1920_8_8_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_8_8_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#elif CONFIG_BSP_LCD_TYPE_320_1480_11_9_INCH
    esp_lcd_dpi_panel_config_t dpi_config = DSI_PANEL_DPI_11_9_INCH_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#endif
#endif
    dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;

    dsi_vendor_config_t vendor_config = {
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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dsi(io, &lcd_dev_config, &disp_panel), err, TAG, "New LCD panel Waveshare failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");
#endif

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
        .rst_gpio_num = BSP_LCD_TOUCH_RST, // Shared with LCD reset
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH || CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
#elif CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
#elif CONFIG_BSP_LCD_TYPE_480_640_2_8_INCH
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 1,
#elif CONFIG_BSP_LCD_TYPE_480_800_4_INCH
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
#else
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
#endif
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
#if CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH || CONFIG_BSP_LCD_TYPE_800_1280_10_1_INCH_A || CONFIG_BSP_LCD_TYPE_800_1280_8_INCH_A || CONFIG_BSP_LCD_TYPE_720_1280_7_INCH_A
    .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
#else
    .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP,
#endif
    .control_phase_bytes = 1,
    .dc_bit_offset = 0,
    .lcd_cmd_bits = 16,
    .flags = {
        .disable_control_phase = 1,
    }
};
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
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
#if LVGL_VERSION_MAJOR >= 9
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .color_format = LV_COLOR_FORMAT_RGB888,
#else
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
#endif
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .sw_rotate = false,                /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    const lvgl_port_display_dsi_cfg_t dpi_cfg = {
        .flags = {
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

    return lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    esp_lcd_touch_handle_t tp;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
            .buff_dma = false,
#else
            .buff_dma = true,
#endif
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

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
