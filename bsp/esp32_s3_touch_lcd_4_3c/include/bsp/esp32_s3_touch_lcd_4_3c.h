#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/i2s_std.h"
#include "driver/temperature_sensor.h"
#include "custom_io_expander_ch32v003.h"
#include "pcf85063a.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "esp_codec_dev.h"

#include "lvgl.h"
#include "esp_lv_adapter.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        0
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 * ESP-SparkBot-BSP pinout
 **************************************************************************************************/
/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL             (GPIO_NUM_9)
#define BSP_I2C_SDA             (GPIO_NUM_8)

/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_SCLK            (GPIO_NUM_44)
#define BSP_I2S_MCLK            (GPIO_NUM_6)
#define BSP_I2S_LCLK            (GPIO_NUM_16)
#define BSP_I2S_DOUT            (GPIO_NUM_15)    // To Codec ES8311
#define BSP_I2S_DSIN            (GPIO_NUM_43)   // From ADC ES7210
#define BSP_POWER_AMP_IO        (IO_EXPANDER_PIN_NUM_3)

/* Display */
#define BSP_LCD_VSYNC     (GPIO_NUM_3)
#define BSP_LCD_HSYNC     (GPIO_NUM_46)
#define BSP_LCD_DE        (GPIO_NUM_5)
#define BSP_LCD_PCLK      (GPIO_NUM_7)
#define BSP_LCD_DISP      (GPIO_NUM_NC)

// Blue data signals
#define BSP_LCD_DATA0        (GPIO_NUM_14) ///< B3
#define BSP_LCD_DATA1        (GPIO_NUM_38) ///< B4
#define BSP_LCD_DATA2        (GPIO_NUM_18) ///< B5
#define BSP_LCD_DATA3        (GPIO_NUM_17) ///< B6
#define BSP_LCD_DATA4        (GPIO_NUM_10) ///< B7

// Green data signals
#define BSP_LCD_DATA5        (GPIO_NUM_39) ///< G2
#define BSP_LCD_DATA6        (GPIO_NUM_0)  ///< G3
#define BSP_LCD_DATA7        (GPIO_NUM_45) ///< G4
#define BSP_LCD_DATA8        (GPIO_NUM_48) ///< G5
#define BSP_LCD_DATA9        (GPIO_NUM_47) ///< G6
#define BSP_LCD_DATA10       (GPIO_NUM_21) ///< G7

// Red data signals
#define BSP_LCD_DATA11       (GPIO_NUM_1)  ///< R3
#define BSP_LCD_DATA12       (GPIO_NUM_2)  ///< R4
#define BSP_LCD_DATA13       (GPIO_NUM_42) ///< R5
#define BSP_LCD_DATA14       (GPIO_NUM_41) ///< R6
#define BSP_LCD_DATA15       (GPIO_NUM_40) ///< R7


#define BSP_LCD_BACKLIGHT     (IO_EXPANDER_PIN_NUM_2)
#define BSP_LCD_RST           (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_RST     (IO_EXPANDER_PIN_NUM_1)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_4)
/** @} */ // end of display

/* uSD card */
#define BSP_SD_D0            (GPIO_NUM_13)
#define BSP_SD_CMD           (GPIO_NUM_11)
#define BSP_SD_CLK           (GPIO_NUM_12)

/** @} */ // end of uSD card

#define BSP_IO_EXPANDER_I2C_ADDRESS     (CUSTOM_IO_EXPANDER_I2C_CH32V003_ADDRESS)
#define BSP_RTC_I2C_ADDRESS     (PCF85063A_ADDRESS)

#define LVGL_BUFFER_HEIGHT          (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2C interface
 *
 **************************************************************************************************/
#define BSP_I2C_NUM     CONFIG_BSP_I2C_NUM

/**
 * @brief Init I2C driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_deinit(void);

/**
 * @brief Get I2C driver handle
 *
 * @return
 *      - I2C handle
 *
 */
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output(playback) and input(recording) path
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Init audio
 *
 * @note There is no deinit audio function. Users can free audio resources by calling i2s_del_channel()
 * @warning The type of i2s_config param is depending on IDF version.
 * @param[in]  i2s_config I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 *      - ESP_ERR_NO_MEM        No memory for storing the channel information
 *      - ESP_ERR_INVALID_STATE This channel has not initialized or already started
 */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);

/**
 * @brief Initialize speaker codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

/**
 * @brief Initialize microphone codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

/**
 * @brief Enable/disable audio power amplifier (deprecated)
 *
 * @param[in] enable: Enable/disable audio power amplifier
 *
 * @return
 *      - ESP_OK:               On success
 *      - ESP_ERR_INVALID_ARG:  Invalid GPIO number
 */
esp_err_t bsp_audio_poweramp_enable(bool enable);

/**************************************************************************************************
 *
 * SPIFFS
 *
 * After mounting the SPIFFS, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SPIFFS_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello World!\n");
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SPIFFS_MOUNT_POINT      CONFIG_BSP_SPIFFS_MOUNT_POINT

/**
 * @brief Mount SPIFFS to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_register was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
 */
esp_err_t bsp_spiffs_mount(void);

/**
 * @brief Unmount SPIFFS from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if already unmounted
 */
esp_err_t bsp_spiffs_unmount(void);

/** @} */ // end of storage





/**************************************************************************************************
 *
 * uSD card
 *
 * After mounting the uSD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SD_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT
extern sdmmc_card_t *bsp_sdcard;

/**
 * @brief Mount microSD card to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_mount(void);

/**
 * @brief Unmount microSD card from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain FATFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_spiflash_mount was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes from wear levelling library, SPI flash driver, or FATFS drivers
 */
esp_err_t bsp_sdcard_unmount(void);

/** @defgroup g99_others Others
 *  @brief Other BSP API
 *  @{
 */

/**************************************************************************************************
 *
 * IO Expander Interface
 *
 **************************************************************************************************/ 
/**
 * @brief Init Custom IO expander chip CH32V003
 *
 * @note If the device was already initialized, users can also use it to get handle.
 * @note This function will be called in `bsp_display_start()` when using LCD sub-board 2 with the resolution of 480x480.
 * @note This function will be called in `bsp_audio_init()`.
 *
 * @return Pointer to device handle or NULL when error occurred
 */
esp_io_expander_handle_t bsp_io_expander_init(void);

/**
 * @brief Get ADC value from the custom IO chip
 *
 * @param[out] adc_value    Retrieved ADC value （0~1023)
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_get_custom_io_adc(uint16_t *adc_value);

/**
 * @brief Get RTC interrupt information (polling method due to limited resources)
 *
 * @param[out] value   Pointer to store the interrupt status
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_get_rtc_int(uint8_t *value);

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP-SparkBot-BSP is shipped with 1.3inch ST7789 display controller.
 * It features 16-bit colors and 240x240 resolution.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * If you want to use the display without LVGL, see bsp/display.h API and use BSP version with 'noglib' suffix.
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (16 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

/**
 * @brief BSP display configuration structure
 */
typedef struct {
    esp_lv_adapter_config_t          lv_adapter_cfg;
    esp_lv_adapter_rotation_t        rotation;
    esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode;
    struct {
        unsigned int swap_xy;  /*!< Swap X and Y after read coordinates */
        unsigned int mirror_x; /*!< Mirror X after read coordinates */
        unsigned int mirror_y; /*!< Mirror Y after read coordinates */
    } touch_flags;
} bsp_display_cfg_t;

#define BSP_TOUCH_FLAGS_FROM_ROT(_rot)                                      \
    ((_rot) == ESP_LV_ADAPTER_ROTATE_0)   ?                                  \
        (typeof(((bsp_display_cfg_t *)0)->touch_flags)){ .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 } : \
    ((_rot) == ESP_LV_ADAPTER_ROTATE_90)  ?                                  \
        (typeof(((bsp_display_cfg_t *)0)->touch_flags)){ .swap_xy = 1, .mirror_x = 1, .mirror_y = 0 } : \
    ((_rot) == ESP_LV_ADAPTER_ROTATE_180) ?                                  \
        (typeof(((bsp_display_cfg_t *)0)->touch_flags)){ .swap_xy = 0, .mirror_x = 1, .mirror_y = 1 } : \
        /* ROTATE_270 */                                                      \
        (typeof(((bsp_display_cfg_t *)0)->touch_flags)){ .swap_xy = 1, .mirror_x = 0, .mirror_y = 1 }

#define BSP_DISPLAY_CFG_ROT(_cfg,_rot)                                  \
{                                                                  \
    .lv_adapter_cfg = _cfg,             \
    .rotation = (_rot),                                            \
    .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT, \
    .touch_flags = BSP_TOUCH_FLAGS_FROM_ROT(_rot),                 \
}

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start_with_config(bsp_display_cfg_t *cfg);

/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
esp_err_t bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);

/**
 * @brief Set the current PCLK frequency
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] freq_hz PCLK frequency(HZ)
 * 
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_set_display_pclk(uint32_t freq_hz);


/**************************************************************************************************
 *
 * CPU Temperature
 *
 **************************************************************************************************/
/**
 * @brief Initialize CPU temperature sensor
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_cpu_temp_init();

/**
 * @brief Get CPU temperature value
 *
 * @param[out] tsens_value   Pointer to store the temperature value
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_get_cpu_temp(float *tsens_value);


/**************************************************************************************************
 *
 * RTC Interface
 *
 **************************************************************************************************/
/**
 * @brief Initialize PCF85063A RTC
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_rtc_init();

/**
 * @brief Set RTC date and time
 *
 * @param[in] time   RTC date and time structure
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_set_rtc_time_date(pcf85063a_datetime_t time);

/**
 * @brief Set RTC alarm time
 *
 * @param[in] time   RTC alarm time structure
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_set_rtc_alarm_time(pcf85063a_datetime_t time);

/**
 * @brief Get current RTC date and time
 *
 * @param[out] time   Pointer to store RTC date and time structure
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_get_rtc_time_date(pcf85063a_datetime_t *time);

/**
 * @brief Get currently set RTC alarm time
 *
 * @param[out] time   Pointer to store RTC alarm time structure
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_get_rtc_alarm_time(pcf85063a_datetime_t *time);

/**
 * @brief Enable RTC alarm
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_enable_rtc_alarm();

/**
 * @brief Convert RTC datetime structure to string
 *
 * @param[out] datetime_str   Buffer to store formatted datetime string
 * @param[in]  time           RTC datetime structure
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time);

#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

#ifdef __cplusplus
}
#endif
