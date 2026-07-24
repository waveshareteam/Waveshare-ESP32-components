/*
 * SPDX-FileCopyrightText: 2026 Waveshare Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_codec_dev.h"
#include "esp_io_expander_tca9554.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"


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
#define BSP_CAPS_IMU            1
#define BSP_CAPS_RTC            1
#define BSP_CAPS_PMU            1
#define BSP_CAPS_IO_EXPANDER    1

/**************************************************************************************************
 * ESP32-C6-Touch-AMOLED-1.8 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_7)
#define BSP_I2C_SDA           (GPIO_NUM_8)

#define BSP_I2S_SCLK          (GPIO_NUM_20)
#define BSP_I2S_MCLK          (GPIO_NUM_19)
#define BSP_I2S_LCLK          (GPIO_NUM_22)
#define BSP_I2S_DOUT          (GPIO_NUM_23)
#define BSP_I2S_DSIN          (GPIO_NUM_21)
#define BSP_POWER_AMP_IO      (IO_EXPANDER_PIN_NUM_7)

/* Display */
#define BSP_LCD_CS        (GPIO_NUM_5)
#define BSP_LCD_PCLK      (GPIO_NUM_0)
#define BSP_LCD_DATA0     (GPIO_NUM_1)
#define BSP_LCD_DATA1     (GPIO_NUM_2)
#define BSP_LCD_DATA2     (GPIO_NUM_3)
#define BSP_LCD_DATA3     (GPIO_NUM_4)

#define BSP_LCD_BACKLIGHT     (GPIO_NUM_NC)
#define BSP_LCD_RST           (IO_EXPANDER_PIN_NUM_4)
#define BSP_LCD_TOUCH_RST     (IO_EXPANDER_PIN_NUM_5)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_15)

/* uSD card */
#define BSP_SD_MOSI           (GPIO_NUM_10)
#define BSP_SD_SCK            (GPIO_NUM_11)
#define BSP_SD_MISO           (GPIO_NUM_18)
#define BSP_SD_SDCS           (GPIO_NUM_6)

#define BSP_IO_EXPANDER_I2C_ADDRESS     (ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000)
#define BSP_PMU_I2C_ADDRESS             (0x34)
#define BSP_IMU_I2C_ADDRESS             (0x6B)
#define BSP_RTC_I2C_ADDRESS             (0x51)
#define BSP_TOUCH_CST820_I2C_ADDRESS    (0x15)
#define BSP_TOUCH_FT5X06_I2C_ADDRESS    (0x38)

#define LVGL_BUFFER_HEIGHT          (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Supported hardware variants
 */
typedef enum {
    BSP_BOARD_VARIANT_UNKNOWN = 0,
    BSP_BOARD_VARIANT_SH8601_FT5X06,
    BSP_BOARD_VARIANT_CO5300_CST820,
} bsp_board_variant_t;

/**
 * @brief Probe the touch controller and cache the detected board variant
 *
 * CST820 at address 0x15 selects V2. FT5x06 at address 0x38 selects V1.
 * The probe initializes I2C0 and resets the display/touch lines through the TCA9554.
 *
 * @note The result is cached until bsp_i2c_deinit() is called.
 * @note This function must not be called from an ISR.
 * @return Detected variant, or BSP_BOARD_VARIANT_UNKNOWN if no supported touch controller responds
 */
bsp_board_variant_t bsp_board_detect(void);

/**
 * @brief Get the cached board variant without probing
 */
bsp_board_variant_t bsp_board_get_variant(void);

/**
 * @brief Get a human-readable name for a board variant
 */
const char *bsp_board_variant_to_name(bsp_board_variant_t variant);

/**************************************************************************************************
 *
 * I2C interface
 *
 * I2C0 is shared by TCA9554, AXP2101, QMI8658, PCF85063, ES8311 and the
 * detected touch controller. Use bsp_i2c_get_handle() with the public address
 * constants above for devices that do not have a dedicated BSP wrapper.
 **************************************************************************************************/
#define BSP_I2C_NUM     (I2C_NUM_0)
#define BSP_I2S_NUM     (I2S_NUM_0)

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
 * @note The display and SD card use the ESP32-C6 SPI2 peripheral with
 * different pin sets and therefore cannot be active at the same time.
 * Mount the SD card before display initialization only in display-free flows.
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

/**
 * @brief Init IO expander chip TCA9554
 *
 * @note If the device was already initialized, users can also use it to get handle.
 * @note Board detection and audio initialization call this function automatically.
 *
 * @return Pointer to device handle or NULL when error occurred
 */
esp_io_expander_handle_t bsp_io_expander_init(void);


/**************************************************************************************************
 *
 * LCD interface
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * If you want to use the display without LVGL, see bsp/display.h API and use BSP version with 'noglib' suffix.
 **************************************************************************************************/
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)
#define BSP_LCD_DRAW_BUFF_DOUBLE   (0)

/**
 * @brief BSP display configuration structure
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    uint32_t        trans_size;
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
    } flags;
} bsp_display_cfg_t;

/**
 * @brief Initialize display
 *
 * This function automatically detects the board variant, initializes SPI2,
 * display and touch controllers, sets brightness to 100%, and starts LVGL.
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function automatically detects the board variant, initializes SPI2,
 * display and touch controllers, sets brightness to 100%, and starts LVGL.
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

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
bool bsp_display_lock(uint32_t timeout_ms);

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
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

#ifdef __cplusplus
}
#endif
