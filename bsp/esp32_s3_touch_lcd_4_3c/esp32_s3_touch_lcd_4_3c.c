#include <stdio.h>
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_io.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_lcd_touch_gt911.h"

#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_codec_dev_defaults.h"
#include "bsp/esp32_s3_touch_lcd_4_3c.h"
#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp_err_check.h"

static const char *TAG = "ESP32-S3-Touch-LCD-4.3C";

static bool gpio_initialized = false;

static i2c_master_bus_handle_t i2c_handle = NULL;  // I2C Handle
static bool i2c_initialized = false;
static esp_io_expander_handle_t custom_io_expander = NULL; // Custom IO expander ch32v003 handle
sdmmc_card_t *bsp_sdcard = NULL; // Global uSD card handler
temperature_sensor_handle_t temp_sensor = NULL;
static pcf85063a_dev_t dev;

static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
static esp_lcd_touch_handle_t tp = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;           // LCD panel handle
uint8_t brightness;
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

#define LCD_BRIGHTNESS_MAX 0xFF 
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
    io_conf.pin_bit_mask = 1ULL << BSP_LCD_TOUCH_INT;    // Select the GPIO pin using a bitmask
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

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
        .glitch_ignore_cnt = 7,
        // .flags.enable_internal_pullup = true,
        // .trans_queue_depth = 0,
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
    bsp_i2c_init();
    return i2c_handle;
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

#if !CONFIG_FATFS_LONG_FILENAMES
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
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(custom_io_expander, BSP_POWER_AMP_IO, (uint8_t)enable));
    return ESP_OK;
}

 esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
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
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
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
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_cfg), err, TAG, "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "I2S enabling failed");
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK_GOTO(i2s_data_if, err);

    bsp_io_expander_init();
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(custom_io_expander, BSP_POWER_AMP_IO, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(custom_io_expander, BSP_POWER_AMP_IO, true));

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

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_data_if == NULL)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
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

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_data_if == NULL)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
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
esp_err_t bsp_display_brightness_init(void)
{
    custom_io_expander_set_pwm(custom_io_expander, LCD_BRIGHTNESS_MAX);
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    } else if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    int flipped_brightness = 100 - brightness_percent;

    brightness = (uint8_t)flipped_brightness;

    custom_io_expander_set_pwm(custom_io_expander, brightness * LCD_BRIGHTNESS_MAX / 100);

    return ESP_OK;
}
int bsp_display_brightness_get(void)
{
    return brightness;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_set_display_pclk(uint32_t freq_hz)
{
    return esp_lcd_rgb_panel_set_pclk(panel_handle,freq_hz);;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_io_expander_handle_t expander = NULL;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    BSP_NULL_CHECK(expander = bsp_io_expander_init(), ESP_FAIL);

    esp_io_expander_set_dir(custom_io_expander, BSP_LCD_BACKLIGHT | BSP_LCD_TOUCH_RST , IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(custom_io_expander, BSP_LCD_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(BSP_LCD_TOUCH_INT, 0); 
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_io_expander_set_level(custom_io_expander, BSP_LCD_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_lcd_rgb_panel_config_t rgb_config = {
           .clk_src = LCD_CLK_SRC_PLL160M,
           .psram_trans_align = 64,
           .data_width = BSP_RGB_DATA_WIDTH,
           .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
           .de_gpio_num = BSP_LCD_DE,
           .pclk_gpio_num = BSP_LCD_PCLK,
           .vsync_gpio_num = BSP_LCD_VSYNC,
           .hsync_gpio_num = BSP_LCD_HSYNC,
           .disp_gpio_num = BSP_LCD_DISP,
           .data_gpio_nums = {
               BSP_LCD_DATA0,
               BSP_LCD_DATA1,
               BSP_LCD_DATA2,
               BSP_LCD_DATA3,
               BSP_LCD_DATA4,
               BSP_LCD_DATA5,
               BSP_LCD_DATA6,
               BSP_LCD_DATA7,
               BSP_LCD_DATA8,
               BSP_LCD_DATA9,
               BSP_LCD_DATA10,
               BSP_LCD_DATA11,
               BSP_LCD_DATA12,
               BSP_LCD_DATA13,
               BSP_LCD_DATA14,
               BSP_LCD_DATA15,
           },
           .timings = BSP_LCD_800_480_PANEL_35HZ_RGB_TIMING(),
           .flags.fb_in_psram = 1,
           .num_fbs = CONFIG_BSP_LCD_RGB_BUFFER_NUMS,
// #if !CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
//            .bounce_buffer_size_px = BSP_LCD_DRAW_BUFF_SIZE,
// #endif
    };

    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_rgb_panel(&rgb_config, &panel_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_init(panel_handle));

    if (ret_panel) {
        *ret_panel = panel_handle;
    }

    if (ret_io) {
        *ret_io = io_handle;
    }

    return ESP_OK;
}


esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    if (!i2c_initialized)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    }
    
    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
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
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
    .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
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

static lv_display_t *bsp_display_lcd_init()
{
    esp_lcd_panel_io_handle_t io_handle = NULL;

    bsp_display_config_t disp_config = { 0 };

    bsp_io_expander_init();

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&disp_config, &panel_handle, &io_handle));

    int buffer_size = 0;
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES;
#else
    buffer_size = BSP_LCD_H_RES * LVGL_BUFFER_HEIGHT;
#endif /* CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR */

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = buffer_size,

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
            .buff_dma = false,
#if CONFIG_BSP_DISPLAY_LVGL_PSRAM
            .buff_spiram = false,
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = 1,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = 1,
#endif
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = false,
#endif
        }
    };
    
    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
            .bb_mode = 1,
#else
            .bb_mode = 0,
#endif
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
    ESP_LOGW(TAG, "CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE");
#endif

    return lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
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


 lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG()
    };

    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg)); /* lvgl task, tick etc*/

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

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
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

/**************************************************************************************************
 *
 * IO Expander Function
 *
 **************************************************************************************************/
esp_io_expander_handle_t bsp_io_expander_init()
{
    if (!i2c_initialized)
    {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    }
    if (!custom_io_expander) {
        BSP_ERROR_CHECK_RETURN_NULL(custom_io_expander_new_i2c_ch32v003(i2c_handle, BSP_IO_EXPANDER_I2C_ADDRESS, &custom_io_expander));
    }
    return custom_io_expander;
}

esp_err_t bsp_get_custom_io_adc(uint16_t *adc_value)
{
    float value = 0; // Variable to store the ADC reading
    uint16_t temp;
    // Take 10 ADC readings and average them to reduce noise
    for (int i = 0; i < 10; i++)
    {
        custom_io_expander_get_adc(custom_io_expander, &temp);
        value += temp; // Read the ADC input
        vTaskDelay(20); // Delay between readings
    }
    value /= 10.0; // Calculate the average value
    value *= 3 * 3.3 / 1023.0;
    if (value > 4.2)
    {
        value = 4.2;
    }
    *adc_value = value * 1000;
    return ESP_OK;
}

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

esp_err_t bsp_get_rtc_int(uint8_t *value)
{
    custom_io_expander_get_int(custom_io_expander, value);
    return ESP_OK;
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
