#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "esp_codec_dev.h"
#include "sdkconfig.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lv_adapter.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

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
 *  ESP-BOX pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_8)
#define BSP_I2C_SDA           (GPIO_NUM_7)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_12)
#define BSP_I2S_MCLK          (GPIO_NUM_13)
#define BSP_I2S_LCLK          (GPIO_NUM_10)
#define BSP_I2S_DOUT          (GPIO_NUM_9)
#define BSP_I2S_DSIN          (GPIO_NUM_11)
#define BSP_POWER_AMP_IO      (GPIO_NUM_53)

#define BSP_LCD_BACKLIGHT     (GPIO_NUM_26)
#define BSP_LCD_RST           (GPIO_NUM_27)
#define BSP_LCD_TOUCH_RST     (GPIO_NUM_23)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_NC)

/* uSD card */
#define BSP_SD_D0             (GPIO_NUM_39)
#define BSP_SD_D1             (GPIO_NUM_40)
#define BSP_SD_D2             (GPIO_NUM_41)
#define BSP_SD_D3             (GPIO_NUM_42)
#define BSP_SD_CMD            (GPIO_NUM_44)
#define BSP_SD_CLK            (GPIO_NUM_43)

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - LCD Touch controller
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

/*
 * ES7210 physical microphone masks for mic_selected and esp_codec_dev_set_in_channel_gain().
 * Board labels from MSB to LSB are MIC4=NA, MIC3=RE, MIC2=FR, and MIC1=FL.
 * These masks address physical MIC inputs, not serialized TDM slots.
 */
#define BSP_AUDIO_ES7210_MIC1_FL_MASK          (1U << 0)
#define BSP_AUDIO_ES7210_MIC2_FR_MASK          (1U << 1)
#define BSP_AUDIO_ES7210_MIC3_RE_MASK          (1U << 2)
#define BSP_AUDIO_ES7210_MIC4_NA_MASK          (1U << 3)
#define BSP_AUDIO_ES7210_MIC_MASK_FL_FR        (BSP_AUDIO_ES7210_MIC1_FL_MASK | BSP_AUDIO_ES7210_MIC2_FR_MASK)
#define BSP_AUDIO_ES7210_CONNECTED_MIC_MASK    (BSP_AUDIO_ES7210_MIC_MASK_FL_FR | BSP_AUDIO_ES7210_MIC3_RE_MASK)

/*
 * ES7210 serialized TDM order verified on hardware: ch1, ch3, ch2, ch4.
 * Use these slot masks only with esp_codec_dev_sample_info_t.channel_mask.
 * Do not reuse the physical MIC masks above as TDM slot masks.
 */
#define BSP_AUDIO_TDM_SLOT_COUNT               (4U)
#define BSP_AUDIO_TDM_SLOT_FL                  (0U)  /* slot0 = MIC1 = FL */
#define BSP_AUDIO_TDM_SLOT_RE                  (1U)  /* slot1 = MIC3 = RE */
#define BSP_AUDIO_TDM_SLOT_FR                  (2U)  /* slot2 = MIC2 = FR */
#define BSP_AUDIO_TDM_SLOT_NA                  (3U)  /* slot3 = MIC4 = not connected */
#define BSP_AUDIO_TDM_SLOT_MASK_FL             ESP_CODEC_DEV_MAKE_CHANNEL_MASK(BSP_AUDIO_TDM_SLOT_FL)
#define BSP_AUDIO_TDM_SLOT_MASK_RE             ESP_CODEC_DEV_MAKE_CHANNEL_MASK(BSP_AUDIO_TDM_SLOT_RE)
#define BSP_AUDIO_TDM_SLOT_MASK_FR             ESP_CODEC_DEV_MAKE_CHANNEL_MASK(BSP_AUDIO_TDM_SLOT_FR)
#define BSP_AUDIO_TDM_SLOT_MASK_NA             ESP_CODEC_DEV_MAKE_CHANNEL_MASK(BSP_AUDIO_TDM_SLOT_NA)
#define BSP_AUDIO_TDM_SLOT_MASK_FL_FR          (BSP_AUDIO_TDM_SLOT_MASK_FL | BSP_AUDIO_TDM_SLOT_MASK_FR)

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output (playback)
 *  - ADC ES7210 for input (recording)
 *
 * Codec device handles returned by this BSP are BSP-owned. Applications may open and close them,
 * but must not call esp_codec_dev_delete(). bsp_audio_deinit() closes and deletes the codec devices,
 * their interfaces, the shared data interface, and the I2S channels.
 **************************************************************************************************/

/**
 * @brief Initialize legacy STD TX and STD RX audio
 *
 * Repeated calls return ESP_OK only while the existing audio mode is STD TX and STD RX.
 *
 * @param[in] i2s_config I2S STD configuration. Pass NULL to use mono, duplex, 16-bit, 22050 Hz.
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   Invalid configuration
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 *      - ESP_ERR_NO_MEM        No memory for storing the channel information
 *      - ESP_ERR_INVALID_STATE Audio is already initialized in another mode
 */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);

/**
 * @brief Initialize audio with STD TX and TDM RX
 *
 * TX and RX share the physical clock signals and must use the same sample rate. This function must
 * be called before creating codec devices. It returns ESP_ERR_INVALID_STATE if any audio mode is
 * already initialized, including an earlier mixed-mode initialization.
 *
 * @param[in] tx_config Non-NULL I2S STD configuration for playback
 * @param[in] rx_config Non-NULL I2S TDM configuration for recording
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   NULL configuration or mismatched TX/RX sample rates
 *      - ESP_ERR_INVALID_STATE Audio is already initialized
 *      - other errors from the I2S driver
 */
esp_err_t bsp_audio_init_tx_std_rx_tdm(const i2s_std_config_t *tx_config,
                                       const i2s_tdm_config_t *rx_config);

/**
 * @brief Initialize the 24 kHz voice audio bus profile
 *
 * The physical bus uses 24 kHz, 16-bit stereo STD for TX and 24 kHz, 16-bit TDM slots 0..3 for RX.
 * RX keeps four total slots, BCLK divider 8, and MCLK x256. The hardware-verified serialized order
 * is slot0=MIC1/FL, slot1=MIC3/RE, slot2=MIC2/FR, and slot3=MIC4/NA. The ES7210 enables connected
 * MIC1, MIC2, and MIC3 inputs; the unconnected MIC4 input remains disabled.
 *
 * @note This configures the physical bus only and does not imply AEC or a reference channel.
 * @note esp_codec_dev_sample_info_t.channel is the physical TDM slot count, not the number of
 *       selected PCM channels. Keep channel=BSP_AUDIO_TDM_SLOT_COUNT.
 * @note For FL-only capture, use channel_mask=BSP_AUDIO_TDM_SLOT_MASK_FL. For FL+FR capture, use
 *       channel_mask=BSP_AUDIO_TDM_SLOT_MASK_FL_FR. These resolve to slot0 and slot0|slot2.
 * @note Input gain uses the physical MIC namespace instead: FL is BSP_AUDIO_ES7210_MIC1_FL_MASK and
 *       FL+FR is BSP_AUDIO_ES7210_MIC_MASK_FL_FR. Physical MIC masks and TDM slot masks must not be
 *       interchanged even when an individual bit value happens to match.
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_STATE Audio is already initialized
 *      - other errors from bsp_audio_init_tx_std_rx_tdm()
 */
esp_err_t bsp_audio_init_voice_24k(void);

/**
 * @brief Deinitialize BSP-owned audio resources
 *
 * This function closes and deletes codec devices created by bsp_audio_codec_speaker_init() and
 * bsp_audio_codec_microphone_init(), deletes their interfaces and the shared data interface,
 * disables and deletes RX/TX channels, and clears all internal audio pointers.
 *
 * @warning Stop all tasks using the codec handles and close the devices before calling this function.
 *          All codec handles returned by the BSP become invalid after this call.
 * @note The shared I2C bus is not deinitialized.
 *
 * @return
 *      - ESP_OK On success or when audio is already deinitialized
 *      - ESP_FAIL If a codec interface could not be closed or deleted
 *      - other errors from the I2S driver
 */
esp_err_t bsp_audio_deinit(void);

/**
 * @brief Initialize or get the BSP-owned speaker codec device
 *
 * If audio is not initialized, this function initializes the legacy STD/STD mode.
 *
 * @return BSP-owned codec device handle, or NULL when an error occurs
 */
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

/**
 * @brief Initialize or get the BSP-owned microphone codec device
 *
 * If audio is not initialized, this function initializes the legacy STD/STD mode. In mixed
 * STD TX/TDM RX mode, the ES7210 enables connected physical inputs MIC1, MIC2, and MIC3 while the
 * four-slot TDM frame remains intact. esp_codec_dev_open() selects packed TDM slots through
 * esp_codec_dev_sample_info_t.channel_mask.
 *
 * @return BSP-owned codec device handle, or NULL when an error occurs
 */
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

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

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP-BOX is shipped with 2.4inch ST7789 display controller.
 * It features 16-bit colors, 320x240 resolution and capacitive touch controller.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_MHZ     (80)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    esp_lv_adapter_config_t          lv_adapter_cfg;   /*!< LVGL adapter configuration */
    esp_lv_adapter_rotation_t        rotation;         /*!< Display rotation */
    esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode;  /*!< Tearing avoidance mode */
    struct {
        unsigned int swap_xy: 1;   /*!< Swap X and Y after reading touch coordinates */
        unsigned int mirror_x: 1;  /*!< Mirror X after reading touch coordinates */
        unsigned int mirror_y: 1;  /*!< Mirror Y after reading touch coordinates */
    } touch_flags;
} bsp_display_cfg_t;

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occured
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
 * @return Pointer to LVGL display or NULL when error occured
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
 * @param timeout_ms Timeout in [ms]. -1 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(int32_t timeout_ms);

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

#ifdef __cplusplus
}
#endif
