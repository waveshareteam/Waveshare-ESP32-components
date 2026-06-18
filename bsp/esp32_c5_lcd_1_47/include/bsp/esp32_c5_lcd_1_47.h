/*
 * SPDX-FileCopyrightText: 2026-2027 Waveshare Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP32-C5-LCD-1.47
 */

#pragma once

#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "led_strip.h"

#include "bsp/config.h"
#include "bsp/display.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#define BSP_CAPS_DISPLAY 1
#define BSP_CAPS_TOUCH 0
#define BSP_CAPS_BUTTONS 0
#define BSP_CAPS_AUDIO 0
#define BSP_CAPS_AUDIO_SPEAKER 0
#define BSP_CAPS_AUDIO_MIC 0
#define BSP_CAPS_SDCARD 1
#define BSP_CAPS_IMU 0
#define BSP_CAPS_WS2812 1
/** @} */ // end of capabilities

/**************************************************************************************************
 *  ESP32-C5-LCD-1.47 Pinout
 **************************************************************************************************/
/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL (GPIO_NUM_NC)
#define BSP_I2C_SDA (GPIO_NUM_NC)

/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_SCLK (GPIO_NUM_NC)
#define BSP_I2S_MCLK (GPIO_NUM_NC)
#define BSP_I2S_LCLK (GPIO_NUM_NC)
#define BSP_I2S_DOUT (GPIO_NUM_NC) // To Codec ES8311
#define BSP_I2S_DSIN (GPIO_NUM_NC) // From ADC ES7210
#define BSP_POWER_AMP_IO (GPIO_NUM_NC)
/** @} */ // end of audio

/** @defgroup g04_display Display and Touch
 *  @brief Display BSP API
 *  @{
 */
/* Display */
#define BSP_LCD_CS   (GPIO_NUM_23) 
#define BSP_LCD_CLK  (GPIO_NUM_7)
#define BSP_LCD_DATA (GPIO_NUM_6)
#define BSP_LCD_DC   (GPIO_NUM_24)

#define BSP_LCD_BACKLIGHT (GPIO_NUM_10)
#define BSP_LCD_RST       (GPIO_NUM_26)

/** @} */ // end of display

/* uSD card */
#define BSP_SD_D0  (GPIO_NUM_5)
#define BSP_SD_CMD (GPIO_NUM_6)
#define BSP_SD_CLK (GPIO_NUM_7)
#define BSP_SD_CS  (GPIO_NUM_4)

/** @} */ // end of uSD card

/* WS2812B */
#define BSP_WS2812_PIN           (GPIO_NUM_8)
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically

/** @} */ // end of WS2812

#ifdef __cplusplus
extern "C"
{
#endif

/** \addtogroup g01_i2c
 *  @{
 */

/**************************************************************************************************
 *
 * I2C Interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - ADC ES7210 (configuration only)
 *  - LCD Touch controller
 *  - IO expander chip TCA9554
 **************************************************************************************************/
#define BSP_I2C_NUM (CONFIG_BSP_I2C_NUM)

    /**
     * @brief Init I2C driver
     *
     * @return
     *      - ESP_OK:               On success
     *      - ESP_ERR_INVALID_ARG:  I2C parameter error
     *      - ESP_FAIL:             I2C driver installation error
     *
     */
    esp_err_t bsp_i2c_init(void);

    /**
     * @brief Deinit I2C driver and free its resources
     *
     * @return
     *      - ESP_OK:               On success
     *      - ESP_ERR_INVALID_ARG:  I2C parameter error
     *
     */
    esp_err_t bsp_i2c_deinit(void);

    /**
     * @brief Get I2C driver handle
     *
     * @return
     *      - I2C handle
     */
    i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/** @} */ // end of i2c

/** @defgroup g02_storage SD Card and SPIFFS
 *  @brief SPIFFS and SD card BSP API
 *  @{
 */

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
#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT

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
     *      - ESP_ERR_NOT_FOUND if the partition table does not contain SPIFFS partition with given label
     *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_unregister was already called
     *      - ESP_ERR_NO_MEM if memory can not be allocated
     *      - ESP_FAIL if partition can not be mounted
     *      - other error codes
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
#define BSP_SD_MOUNT_POINT CONFIG_BSP_SD_MOUNT_POINT
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
 * LCD interface
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * If you want to use the display without LVGL, see bsp/display.h API and use BSP version with 'noglib' suffix.
 **************************************************************************************************/
#define BSP_LCD_SPI_NUM            (SPI2_HOST)
#define BSP_LCD_SPI_CLK_HZ         (40 * 1000 * 1000)

#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT)
#define BSP_LCD_DRAW_BUFF_DOUBLE   (0)

    /**
     * @brief BSP display configuration structure
     *
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
     * @note This function initializes display controller and starts LVGL handling task.
     * @note Users can get LCD panel handle from `user_data` in returned display.
     *
     * @return Pointer to LVGL display or NULL when error occurred
     */
    lv_display_t *bsp_display_start(void);

    /**
     * @brief Initialize display
     *
     * This function initializes SPI, display controller and starts LVGL handling task.
     * LCD backlight must be enabled separately by calling `bsp_display_brightness_set()`
     *
     * @param cfg display configuration
     *
     * @return Pointer to LVGL display or NULL when error occurred
     */
    lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

    /**
     * @brief Get pointer to input device (touch, buttons, ...)
     *
     * @note  The LVGL input device is initialized in `bsp_display_start()` function.
     * @note  This function should be called after calling `bsp_display_start()`.
     *
     * @return Pointer to LVGL input device or NULL when not initialized
     */
    lv_indev_t *bsp_display_get_input_dev(void);

    /**
     * @brief Take LVGL mutex
     *
     * @note  Display must be already initialized by calling `bsp_display_start()`
     *
     * @param[in] timeout_ms: Timeout in [ms]. 0 will block indefinitely.
     *
     * @return
     *      - true:  Mutex was taken
     *      - false: Mutex was NOT taken
     */
    bool bsp_display_lock(uint32_t timeout_ms);

    /**
     * @brief Give LVGL mutex
     *
     * @note  Display must be already initialized by calling `bsp_display_start()`
     *
     */
    void bsp_display_unlock(void);

    /**
     * @brief Rotate screen
     *
     * @note  Display must be already initialized by calling `bsp_display_start()`
     * @note  This function can't work with the anti-tearing function. Please use the `BSP_DISPLAY_LVGL_ROTATION` configuration instead.
     *
     * @param[in] disp:     Pointer to LVGL display
     * @param[in] rotation: Angle of the display rotation
     */
    void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation);

    /**
     * @brief Initialize the WS2812B LED strip
     *
     * This function initializes the WS2812B driver and prepares the LED strip
     * for subsequent color control.
     *
     * @return
     *      - ESP_OK: Initialization succeeded.
     *      - ESP_FAIL: Initialization failed.
     */
    esp_err_t bsp_ws2812b_init();

    /**
     * @brief Set the color of a specific WS2812B LED
     *
     * This function sets the color of the LED at the given index in the LED strip.
     *
     * @param[in] index   The LED index to set (0-based).
     * @param[in] g       Green component (0-255).
     * @param[in] r       Red component (0-255).
     * @param[in] b       Blue component (0-255).
     *
     * @return
     *      - ESP_OK: Color set successfully.
     *      - ESP_ERR_INVALID_ARG: Invalid LED index or other parameters.
     *      - ESP_FAIL: Failed to update LED color.
     */
    esp_err_t bsp_setledcolor(int index, uint8_t red, uint8_t green, uint8_t blue);


#ifdef __cplusplus
}
#endif
