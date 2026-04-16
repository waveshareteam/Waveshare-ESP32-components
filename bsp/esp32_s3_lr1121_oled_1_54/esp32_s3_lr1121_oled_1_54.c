#include <stdio.h>
#include "esp_lcd_panel_io.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_codec_dev_defaults.h"
#include "bsp/esp32_s3_lr1121_oled_1_54.h"
#include "bsp/buttons.h"
#include "bsp_err_check.h"
#include "bsp_err_check.h"

#include "lvgl.h"
#include "esp_lv_adapter.h"

static const char *TAG = "ESP32-S3-LR1121-OLED-1.54";

static bool gpio_initialized = false;

static i2c_bus_handle_t i2c_bus = NULL;
static i2c_master_bus_handle_t i2c_handle = NULL;  // I2C Handle
static bool i2c_initialized = false;

static spi_device_handle_t spi_handle = NULL;// SPI Handle
static bool spi_initialized = false;

static bq27220_handle_t bq27220 = NULL;

sdmmc_card_t *bsp_sdcard = NULL; // Global uSD card handler
temperature_sensor_handle_t temp_sensor = NULL;
static pcf85063a_dev_t dev;

static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;           // LCD panel handle

static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL; /* Codec data interface */

#define BSP_ES7210_CODEC_ADDR ES7210_CODEC_DEFAULT_ADDR
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

#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

/**************************************************************************************************
 *
 * GPIO Function
 *
 **************************************************************************************************/
esp_err_t bsp_gpio_init(void)
{
    // Zero-initialize the GPIO configuration structure
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupts for this pin
    io_conf.pin_bit_mask = 1ULL << BSP_POWER_AMP_IO;    // Select the GPIO pin using a bitmask
    io_conf.mode = GPIO_MODE_OUTPUT;          // Set pin as output
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // Disable pull-up
    gpio_config(&io_conf); // Apply the configuration

    gpio_initialized = true;

    return ESP_OK;
}


/**************************************************************************************************
 *
 * I2C Function
 *
 **************************************************************************************************/
esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400 * 1000,
    };
    i2c_bus = i2c_bus_create(BSP_I2C_NUM, &conf);

    i2c_handle = i2c_bus_get_internal_bus_handle(i2c_bus);

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
    bsp_i2c_init();
    return i2c_handle;
}

/**************************************************************************************************
 *
 * SPI Function
 *
 **************************************************************************************************/
esp_err_t bsp_spi_init(void)
{
    /* SPI was initialized before */
    if (spi_initialized) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = BSP_SPI_MOSI,
        .miso_io_num = BSP_SPI_MISO,
        .sclk_io_num = BSP_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = BSP_SPI_CLK_SPEED_HZ,  // 8 MHz
        .mode = 0,                          // SPI mode 0: CPOL=0, CPHA=0
        .spics_io_num = -1,
        .queue_size = 1,
    };
    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(BSP_SPI_NUM, &devcfg, &spi_handle));

    spi_initialized = true;

    return ESP_OK;
}

spi_device_handle_t bsp_spi_get_handle(void)
{
    if (spi_handle == NULL)
    {
        bsp_spi_init();
    }
    return spi_handle;
}

/**************************************************************************************************
 *
 * SPIFFS Function
 *
 **************************************************************************************************/
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
 * SD Function
 *
 **************************************************************************************************/
esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    const sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    const sdmmc_slot_config_t slot_config = {
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0 = BSP_SD_D0,
        .d1 = GPIO_NUM_NC,
        .d2 = GPIO_NUM_NC,
        .d3 = GPIO_NUM_NC,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };

#if CONFIG_FATFS_LFN_NONE
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

/**************************************************************************************************
 *
 * I2S Audio Function
 *
 **************************************************************************************************/
esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_POWER_AMP_IO, enable));
    return ESP_OK;
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, uint32_t sample_rate)
{
    esp_err_t ret = ESP_FAIL;
    if (i2s_tx_chan && i2s_rx_chan)
    {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(sample_rate);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL)
    {
        p_i2s_cfg = i2s_config;
    }

    if (i2s_tx_chan != NULL)
    {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg), err, TAG, "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, TAG, "I2S enabling failed");
    }
    if (i2s_rx_chan != NULL)
    {

        i2s_tdm_config_t tdm_cfg = {
            .clk_cfg = {
                .sample_rate_hz = sample_rate,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .ext_clk_freq_hz = 0,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
                .bclk_div = 8,
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_STEREO, //I2S_SLOT_MODE_STEREO
                .slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3,
                .ws_width = I2S_TDM_AUTO_WS_WIDTH,
                .ws_pol = false,
                .bit_shift = true,
                .left_align = false,
                .big_endian = false,
                .bit_order_lsb = false,
                .skip_mask = false,
                .total_slot = 4
            },
            .gpio_cfg = {
                .mclk = BSP_I2S_MCLK,  
                .bclk = BSP_I2S_SCLK,  
                .ws = BSP_I2S_LCLK,    
                .dout = BSP_I2S_DOUT,  
                .din = BSP_I2S_DSIN,   
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false
                }
            }
        };
        ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &tdm_cfg));

        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "RX enable failed");

    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK_GOTO(i2s_data_if, err);

    bsp_gpio_init();
    bsp_audio_poweramp_enable(true);

    return ESP_OK;

err:
    if (i2s_tx_chan)
    {
        i2s_del_channel(i2s_tx_chan);
    }
    if (i2s_rx_chan)
    {
        i2s_del_channel(i2s_rx_chan);
    }

    return ret;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(uint32_t sample_rate)
{
    if (i2s_data_if == NULL)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL, sample_rate));
    }
    assert(i2s_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
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
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    BSP_NULL_CHECK(es8311_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(uint32_t sample_rate)
{
    if (i2s_data_if == NULL)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL, sample_rate));
    }
    assert(i2s_data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = BSP_ES7210_CODEC_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = i2c_ctrl_if,
    };
    es7210_cfg.mic_selected = ES7120_SEL_MIC1 | ES7120_SEL_MIC2 | ES7120_SEL_MIC3 | ES7120_SEL_MIC4;    
    const audio_codec_if_t *es7210_dev = es7210_codec_new(&es7210_cfg);
    BSP_NULL_CHECK(es7210_dev, NULL);

    esp_codec_dev_cfg_t codec_es7210_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7210_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_es7210_dev_cfg);
}

/**********************************************************************************************************
 *
 * Display Function
 *
 **********************************************************************************************************/
esp_err_t bsp_display_backlight_off(void)
{
    return esp_lcd_panel_disp_on_off(panel_handle, false);
}

esp_err_t bsp_display_backlight_on(void)
{
    return esp_lcd_panel_disp_on_off(panel_handle, true);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    if (!i2c_initialized)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    }
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = BSP_OLED_I2C_ADDRESS,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = 8,                      // According to SSD1306 datasheet
        .lcd_param_bits = 8,                    // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet

    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &io_config, &io_handle));


    ESP_LOGI(TAG, "Install SSD1309 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = BSP_OLED_RST,
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0))
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
#endif
    };

    esp_lcd_panel_ssd1309_config_t ssd1309_config = {
        .height = 64,
    };
    panel_config.vendor_config = &ssd1309_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1309(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle,false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    if (ret_panel) {
        *ret_panel = panel_handle;
    }

    if (ret_io) {
        *ret_io = io_handle;
    }

    return ESP_OK;
}

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;

    bsp_display_config_t disp_config = { 0 };


    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&disp_config, &panel_handle, &io_handle));

    ESP_LOGI(TAG, "Registering OLED display");
    esp_lv_adapter_display_config_t display_config =  ESP_LV_ADAPTER_DISPLAY_SPI_MONO_DEFAULT_CONFIG(
        panel_handle,           	// LCD panel handle
        io_handle,        			// LCD panel IO handle (can be NULL)
        BSP_LCD_H_RES,             		// Horizontal resolution
        BSP_LCD_V_RES,             	// Vertical resolution
        cfg->rotation, 	// Rotation
        ESP_LV_ADAPTER_MONO_LAYOUT_VTILED 	// Vertical tiled layout
    );
    
    return esp_lv_adapter_register_display(&display_config);
}

static button_handle_t prev_btn_handle = NULL, next_btn_handle = NULL, enter_btn_handle = NULL;
esp_err_t bsp_buttons_new(const bsp_display_cfg_t *cfg, esp_buttons_handle_t *ret_buttons)
{
    const button_gpio_config_t bsp_button_config[] = {
        {
            .gpio_num = BOOT_KEY_GPIO,
            .active_level = 0,
        },
        {
            .gpio_num = MENU_KEY_GPIO,
            .active_level = 0,
        },
        {
            .gpio_num = PTT_KEY_GPIO,
            .active_level = 0,
        },
    };

    const button_config_t btn_cfg = {0};
    /* Create button devices */
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[0], &prev_btn_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[1], &next_btn_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[2], &enter_btn_handle);
    
    ESP_LOGI(TAG, "Registering buttons");
    return ret;
}

static lv_indev_t *bsp_display_indev_init(const bsp_display_cfg_t *cfg, lv_display_t *disp)
{
    esp_buttons_handle_t button;
    
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(bsp_buttons_new(cfg, &button));
    // assert(button);

    const esp_lv_adapter_nav_buttons_config_t buttons_cfg = {
        .disp = disp,
        .button_prev = prev_btn_handle,
        .button_next = next_btn_handle,
        .button_enter = enter_btn_handle,
    };
    disp_indev = esp_lv_adapter_register_navigation_buttons(&buttons_cfg);
    lv_indev_set_mode(disp_indev, LV_INDEV_MODE_TIMER); 

    return disp_indev;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_DEFAULT,
        .touch_flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0}};

    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    BSP_ERROR_CHECK_RETURN_NULL(esp_lv_adapter_init(&cfg->lv_adapter_cfg));

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(cfg, disp), NULL);
      
    // bsp_display_rotate(disp, LV_DISPLAY_ROTATION_0);
    ESP_ERROR_CHECK(esp_lv_adapter_start());
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    esp_lv_adapter_unlock();
}
/**************************************************************************************************
 *
 * PCF85063A RTC Function
 *
 **************************************************************************************************/
esp_err_t bsp_rtc_init()
{
    if (!i2c_initialized)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    }
    esp_err_t ret = pcf85063a_init(&dev, i2c_handle, BSP_RTC_I2C_ADDRESS);
    return ret;
}

esp_err_t bsp_set_rtc_time_date(pcf85063a_datetime_t time)
{
    esp_err_t ret = pcf85063a_set_time_date(&dev, time);
    return ret;
}

esp_err_t bsp_set_rtc_alarm_time(pcf85063a_datetime_t time)
{
    esp_err_t ret = pcf85063a_set_alarm(&dev, time);
    return ret;
}

esp_err_t bsp_get_rtc_time_date(pcf85063a_datetime_t *time)
{
    esp_err_t ret = pcf85063a_get_time_date(&dev, time);
    return ret;
}

esp_err_t bsp_get_rtc_alarm_time(pcf85063a_datetime_t *time)
{
    esp_err_t ret = pcf85063a_get_alarm(&dev, time);
    return ret;
}

esp_err_t bsp_enable_rtc_alarm()
{
    esp_err_t ret = pcf85063a_enable_alarm(&dev);
    return ret;
}

esp_err_t bsp_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time)
{
    pcf85063a_datetime_to_str(datetime_str, time);
    return ESP_OK;
}

/**************************************************************************************************
 *
 * BQ27220 BAT Function
 *
 **************************************************************************************************/
esp_err_t bsp_bat_init(uint16_t mah)
{
    if (!i2c_initialized)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    }
    
    static parameter_cedv_t default_cedv = {
        .full_charge_cap = 650,
        .design_cap = 650,
        .reserve_cap = 0,
        .near_full = 97,
        .self_discharge_rate = 10,
        .EDV0 = 3000,
        .EDV1 = 3150,
        .EDV2 = 3300,
        .EMF = 3750,
        .C0 = 140,
        .R0 = 1200,
        .T0 = 4200,
        .R1 = 5200,
        .TC = 11,
        .C1 = 0,
        .DOD0   = 4200,
        .DOD10  = 4100,
        .DOD20  = 4000,
        .DOD30  = 3920,
        .DOD40  = 3850,
        .DOD50  = 3800,
        .DOD60  = 3750,
        .DOD70  = 3700,
        .DOD80  = 3600,
        .DOD90  = 3400,
        .DOD100 = 3000,
    };
    default_cedv.full_charge_cap = mah;
    default_cedv.design_cap = mah;
    static const gauging_config_t default_config = {
        .CCT = 1,
        .CSYNC = 0,
        .EDV_CMP = 0,
        .SC = 1,
        .FIXED_EDV0 = 0,
        .FCC_LIM = 1,
        .FC_FOR_VDQ = 1,
        .IGNORE_SD = 1,
        .SME0 = 0,
    };

    bq27220_config_t bq27220_cfg = {
        .i2c_bus = i2c_bus,
        .cfg = &default_config,
        .cedv = &default_cedv,
    };

    bq27220 = bq27220_create(&bq27220_cfg);
    return ESP_OK;
}

esp_err_t bsp_get_bat_info(bsp_bat_info_t *bat_info)
{
    bat_info->mv  = bq27220_get_voltage(bq27220);
    bat_info->ma  = bq27220_get_current(bq27220);
    bat_info->mah_rem = bq27220_get_remaining_capacity(bq27220);
    bat_info->mah_fcc = bq27220_get_full_charge_capacity(bq27220);
    bat_info->tc = bq27220_get_temperature(bq27220) / 10 - 273; // Convert from 0.1K to Celsius
    bat_info->cycles = bq27220_get_cycle_count(bq27220);
    bat_info->soc = bq27220_get_state_of_charge(bq27220);
    bat_info->mw = bq27220_get_average_power(bq27220); // in mW
    bat_info->max_load = bq27220_get_maxload_current(bq27220); // in mA
    bat_info->tte = bq27220_get_time_to_empty(bq27220);
    bat_info->ttf = bq27220_get_time_to_full(bq27220);
    bat_info->soh = bq27220_get_state_of_health(bq27220);

    // ESP_LOGI(TAG, "Battery Info - Vol: %dmv, Current: %dmA, Power: %dmW, Remaining Capacity: %dmAh, Full Charge Capacity: %dmAh, Temperature: %dC, Cycle Count: %d, SOC: %d%%, Max Load: %dmA, Time to empty: %dmin, Time to full: %dmin, SOH=%d%%",
    //          bat_info->mv, bat_info->ma, bat_info->mw, bat_info->mah_rem, bat_info->mah_fcc, bat_info->tc, bat_info->cycles, bat_info->soc, bat_info->max_load, bat_info->tte, bat_info->ttf, bat_info->soh);

    return ESP_OK;
}
 /**************************************************************************************************
 *
 * Other Function
 *
 **************************************************************************************************/
esp_err_t bsp_cpu_temp_init()
{
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    
    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    return ESP_OK;
}

esp_err_t bsp_get_cpu_temp(float *tsens_value)
{
    ESP_LOGI(TAG, "Read temperature");
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, tsens_value));
    ESP_LOGI(TAG, "Temperature value %.02f ℃", *tsens_value);

    return ESP_OK;
}