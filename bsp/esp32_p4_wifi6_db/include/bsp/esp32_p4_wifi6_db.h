/**
 * @file
 * @brief ESP BSP: ESP32-P4-WIFI6-DB
 */

#pragma once

#include <stddef.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "driver/i2s_std.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_codec_dev.h"
#include "esp_video_device.h"

#if __has_include(<esp_lcd_jd9365.h>)
#include "esp_lcd_jd9365.h"
#endif

#if __has_include(<esp_lcd_ili9881c.h>)
#include "esp_lcd_ili9881c.h"
#endif

#if __has_include(<esp_lcd_hx8394.h>)
#include "esp_lcd_hx8394.h"
#endif

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lv_adapter.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_ESP32_P4_WIFI6_DB
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        0
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            0
#define BSP_CAPS_CAMERA         1
#define BSP_CAPS_RTC            0
/** @} */ // end of capabilities

/**************************************************************************************************
 *  Board pinout
 **************************************************************************************************/

/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL           (GPIO_NUM_8)
#define BSP_I2C_SDA           (GPIO_NUM_7)
/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_SCLK                   (GPIO_NUM_12)
#define BSP_I2S_MCLK                   (GPIO_NUM_13)
#define BSP_I2S_LRCK                   (GPIO_NUM_10)
#define BSP_I2S_DOUT                   (GPIO_NUM_9)  /*!< ESP32-P4 output to ES8311 DSDIN */
#define BSP_I2S_DIN                    (GPIO_NUM_11) /*!< ESP32-P4 input from ES8311 ASDOUT */
#define BSP_I2S_DEFAULT_SAMPLE_RATE_HZ (48000)       /*!< Sample rate used when bsp_audio_init() is called with NULL */
#define BSP_POWER_AMP_IO               (GPIO_NUM_53)
/** @} */ // end of audio

/** @defgroup g04_display Display and Touch
 *  @brief Display BSP API
 *  @{
 */
#define BSP_LCD_RST                       (GPIO_NUM_NC)
#define BSP_LCD_BACKLIGHT_I2C_ADDRESS     (0x45)
#define BSP_LCD_BACKLIGHT_BRIGHTNESS_REG  (0x96)
#define BSP_LCD_TOUCH_RST                 (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_INT                 (GPIO_NUM_NC)
/** @} */ // end of display

/** @defgroup g12_camera Camera
 *  @brief Camera BSP API
 *  @{
 */
#define BSP_CAMERA_GPIO_XCLK (GPIO_NUM_NC)
#define BSP_CAMERA_RST       (GPIO_NUM_NC)
/** @} */ // end of camera

/** @defgroup g02_storage SD Card and SPIFFS
 *  @brief SPIFFS and SD card BSP API
 *  @{
 */
/* uSD card MMC */
#define BSP_SD_D0             (GPIO_NUM_39)
#define BSP_SD_D1             (GPIO_NUM_40)
#define BSP_SD_D2             (GPIO_NUM_41)
#define BSP_SD_D3             (GPIO_NUM_42)
#define BSP_SD_CMD            (GPIO_NUM_44)
#define BSP_SD_CLK            (GPIO_NUM_43)
#define BSP_SD_VDD_EN         (GPIO_NUM_45)
#define BSP_SD_VDD_EN_ACTIVE_LEVEL (0)
#define BSP_SD_PWR_CTRL_LDO_CHAN (4)

/** @} */ // end of storage

/** @defgroup g07_usb USB
 *  @brief USB BSP API
 *  @{
 */
#define BSP_USB_POS           (GPIO_NUM_NC)
#define BSP_USB_NEG           (GPIO_NUM_NC)
/** @} */ // end of usb

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup boardname
 *  @{
 */

/**
 * @brief Get GPIOs routed to the expansion header
 *
 * @param[out] count Number of GPIOs in the returned array, or NULL if not needed
 *
 * @return Read-only GPIO array owned by the BSP
 */
const gpio_num_t *bsp_get_header_gpios(size_t *count);

/** @} */ // end of boardname

/** \addtogroup g01_i2c
 *  @{
 */

/**************************************************************************************************
 *
 * I2C interface
 *
 * The ES8311 codec, LCD backlight controller, and MIPI-CSI camera SCCB bus
 * share this I2C peripheral.
 **************************************************************************************************/
#define BSP_I2C_NUM           CONFIG_BSP_I2C_NUM
#define BSP_I2C_CLK_SPEED_HZ  CONFIG_BSP_I2C_CLK_SPEED_HZ

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

/** @} */ // end of i2c

/** \addtogroup g03_audio
 *  @{
 */

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * The ES8311 codec provides the playback path and one analog microphone input.
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
 * @note If the I2S channels are already running, passing NULL keeps the current configuration, while
 *       passing a configuration reconfigures the running channels in place. The channels are briefly
 *       disabled during that reconfiguration, so do not call it while a codec device is streaming.
 *       The MCLK multiple recorded for the ES8311 is only applied to codec devices created afterwards.
 * @warning The type of i2s_config param is depending on IDF version.
 * @param[in]  i2s_config I2S configuration. Pass NULL to use default values
 *                        (Mono, duplex, 16bit, BSP_I2S_DEFAULT_SAMPLE_RATE_HZ)
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

/** @} */ // end of audio

/** \addtogroup g02_storage
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
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain SPIFFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_unregister was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
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

/**
 * @brief BSP SD card configuration structure
 */
typedef struct {
    const esp_vfs_fat_sdmmc_mount_config_t *mount;
    sdmmc_host_t *host;
    const sdmmc_slot_config_t *slot;
} bsp_sdcard_cfg_t;

/**
 * @brief Mount microSD card to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC, SDMMC protocol, or FATFS drivers
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
 *      - other error codes from the SDMMC or FATFS drivers
 */
esp_err_t bsp_sdcard_unmount(void);

/**
 * @brief Get SD card handle
 *
 * @return SD card handle
 */
sdmmc_card_t *bsp_sdcard_get_handle(void);

/**
 * @brief Get SD card MMC host config
 *
 * @param slot SD card slot
 * @param config Structure which will be filled
 */
void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config);

/**
 * @brief Get SD card MMC slot config
 *
 * @param slot SD card slot
 * @param config Structure which will be filled
 */
void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config);

/**
 * @brief Mount microSD card to virtual file system (MMC mode)
 *
 * @param cfg SD card configuration
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg);

/** @} */ // end of storage

/** \addtogroup g04_display
 *  @{
 */

/**************************************************************************************************
 *
 * LCD interface
 *
 * This board supports Kconfig-selectable JD9365, ILI9881C, and HX8394
 * MIPI-DSI panels without a hardware reset pin. The backlight and GT911 touch
 * controller share the board I2C bus.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_MHZ     (80)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * 50) // Frame buffer size in pixels
#define BSP_LCD_DRAW_BUFF_DOUBLE   (0)

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    esp_lv_adapter_config_t lv_adapter_cfg;               /*!< LVGL adapter configuration */
    esp_lv_adapter_rotation_t rotation;                   /*!< Display rotation */
    esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode;     /*!< Display tearing avoidance mode */
    struct {
        unsigned int swap_xy: 1;                          /*!< Swap touch X and Y coordinates */
        unsigned int mirror_x: 1;                         /*!< Mirror touch X coordinates */
        unsigned int mirror_y: 1;                         /*!< Mirror touch Y coordinates */
    } touch_flags;
} bsp_display_cfg_t;

/**
 * @brief Initialize display
 *
 * This function initializes MIPI-DSI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes MIPI-DSI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Deinitialize display
 *
 * This function deinitializes MIPI-DSI, display controller and stops LVGL.
 *
 * @param display Pointer to LVGL display
 */
void bsp_display_stop(lv_display_t *display);

/**
 * @brief Get the LVGL touch input device
 *
 * @return Touch input device, or NULL when the display/touch is not initialized
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
 * @brief Enable or disable direct framebuffer drawing
 *
 * @param enable True to bypass LVGL rendering and enable direct framebuffer drawing
 *
 * @return ESP_OK on success, or an error from the LVGL adapter
 */
esp_err_t bsp_display_set_dummy_draw(bool enable);

/**
 * @brief Get the framebuffer currently available for direct drawing
 *
 * @return Writable framebuffer, or NULL when direct drawing is unavailable
 */
void *bsp_display_get_free_frame_buffer(void);

/**
 * @brief Submit a complete directly drawn framebuffer to the display
 *
 * @param frame_buffer Framebuffer previously returned by bsp_display_get_free_frame_buffer()
 *
 * @return ESP_OK on success, or an error from the LVGL adapter
 */
esp_err_t bsp_display_flush_frame_buffer(void *frame_buffer);

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

/** @} */ // end of display

/** \addtogroup g07_usb
 *  @{
 */

/**************************************************************************************************
 *
 * USB
 *
 **************************************************************************************************/

/**
 * @brief Power modes of USB Host connector
 */
typedef enum bsp_usb_host_power_mode_t {
    BSP_USB_HOST_POWER_MODE_USB_DEV, //!< Power from USB DEV port
} bsp_usb_host_power_mode_t;

/**
 * @brief Start USB host
 *
 * This is a one-stop-shop function that will configure the board for USB Host mode
 * and start USB Host library
 *
 * @param[in] mode        USB Host connector power mode (Not used on this board)
 * @param[in] limit_500mA Limit output current to 500mA (Not used on this board)
 * @return
 *     - ESP_OK                 On success
 *     - ESP_ERR_INVALID_ARG    Parameter error
 *     - ESP_ERR_NO_MEM         Memory cannot be allocated
 */
esp_err_t bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA);

/**
 * @brief Stop USB host
 *
 * USB Host lib will be uninstalled and power from connector removed.
 *
 * @return
 *     - ESP_OK              On success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_usb_host_stop(void);

/** @} */ // end of usb

/** @addtogroup g12_camera
 *  @{
 */

/**************************************************************************************************
 *
 * Camera interface
 * Supported camera sensors: OV5647, SC2336
 * More information in display_camera_video example
 *
 **************************************************************************************************/

#define BSP_CAMERA_DEVICE       (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define BSP_CAMERA_ROTATION     (0)

/**
 * @brief BSP camera configuration structure (for future use)
 *
 */
typedef struct {
    uint8_t dummy;
} bsp_camera_cfg_t;

/**
 * @brief Initialize camera
 *
 * Camera sensor initialization.
 */
esp_err_t bsp_camera_start(const bsp_camera_cfg_t *cfg);

/** @} */ // end of camera

#ifdef __cplusplus
}
#endif
