/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP32-S31-Touch-LCD-4.3C
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_codec_dev.h"
#include "esp_vfs_fat.h"
#include "pcf85063a.h"
#include "bsp/bq27220.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "bsp/touch.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lv_adapter.h"
#endif

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

#define BSP_BOARD_ESP32_S31_TOUCH_LCD_4_3C

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY          1
#define BSP_CAPS_TOUCH            1
#define BSP_CAPS_BUTTONS          0
#define BSP_CAPS_KNOB             0
#define BSP_CAPS_AUDIO            1
#define BSP_CAPS_AUDIO_SPEAKER    1
#define BSP_CAPS_AUDIO_MIC        1
#define BSP_CAPS_SDCARD           1
#define BSP_CAPS_LED              1
#define BSP_CAPS_CAMERA           0
#define BSP_CAPS_BAT              1
#define BSP_CAPS_IMU              1
#define BSP_CAPS_HUMITURE         0
#define BSP_CAPS_RTC              1
#define BSP_CAPS_LIGHT_SENSOR     0
#define BSP_CAPS_MAGNETOMETER     0

/**************************************************************************************************
 *  Board pinout
 **************************************************************************************************/

/* I2C */
#define BSP_I2C_SCL               (GPIO_NUM_1)
#define BSP_I2C_SDA               (GPIO_NUM_0)
#define BSP_I2C_NUM               CONFIG_BSP_I2C_NUM

/* Audio: ES8389 */
#define BSP_I2S_MCLK              (GPIO_NUM_NC)
#define BSP_I2S_SCLK              (GPIO_NUM_37)
#define BSP_I2S_LCLK              (GPIO_NUM_38)
#define BSP_I2S_DOUT              (GPIO_NUM_39)  /* MCU TX -> ES8389 DSDIN */
#define BSP_I2S_DSIN              (GPIO_NUM_42)  /* MCU RX <- ES8389 ASDOUT */
#define BSP_POWER_AMP_IO          (GPIO_NUM_57)
#define BSP_ES8389_I2C_ADDRESS_7BIT    (0x10)
#define BSP_ES8389_I2C_ADDRESS         BSP_ES8389_I2C_ADDRESS_7BIT
#define BSP_ES8389_I2C_ADDRESS_8BIT    (BSP_ES8389_I2C_ADDRESS_7BIT << 1)

/* Display: 4.3-inch RGB LCD, 800x480 */
#define BSP_LCD_RGB_VSYNC         (GPIO_NUM_45)
#define BSP_LCD_RGB_HSYNC         (GPIO_NUM_44)
#define BSP_LCD_RGB_DE            (GPIO_NUM_43)
#define BSP_LCD_RGB_PCLK          (GPIO_NUM_40)
#define BSP_LCD_RGB_DISP          (GPIO_NUM_NC)
#define BSP_LCD_RGB_DATA0         (GPIO_NUM_8)   /* B3 */
#define BSP_LCD_RGB_DATA1         (GPIO_NUM_9)   /* B4 */
#define BSP_LCD_RGB_DATA2         (GPIO_NUM_10)  /* B5 */
#define BSP_LCD_RGB_DATA3         (GPIO_NUM_11)  /* B6 */
#define BSP_LCD_RGB_DATA4         (GPIO_NUM_12)  /* B7 */
#define BSP_LCD_RGB_DATA5         (GPIO_NUM_13)  /* G2 */
#define BSP_LCD_RGB_DATA6         (GPIO_NUM_14)  /* G3 */
#define BSP_LCD_RGB_DATA7         (GPIO_NUM_15)  /* G4 */
#define BSP_LCD_RGB_DATA8         (GPIO_NUM_16)  /* G5 */
#define BSP_LCD_RGB_DATA9         (GPIO_NUM_17)  /* G6 */
#define BSP_LCD_RGB_DATA10        (GPIO_NUM_18)  /* G7 */
#define BSP_LCD_RGB_DATA11        (GPIO_NUM_19)  /* R3 */
#define BSP_LCD_RGB_DATA12        (GPIO_NUM_33)  /* R4 */
#define BSP_LCD_RGB_DATA13        (GPIO_NUM_34)  /* R5 */
#define BSP_LCD_RGB_DATA14        (GPIO_NUM_35)  /* R6 */
#define BSP_LCD_RGB_DATA15        (GPIO_NUM_36)  /* R7 */
#define BSP_LCD_RST               (GPIO_NUM_NC)
#define BSP_LCD_BACKLIGHT         (GPIO_NUM_47)
#define BSP_LCD_TOUCH_INT         (GPIO_NUM_2)
#define BSP_LCD_TOUCH_RST         (GPIO_NUM_46)
#define BSP_RGB_DATA_WIDTH        (16)

/* Compatibility aliases used by Waveshare RGB LCD BSPs. */
#define BSP_LCD_VSYNC             BSP_LCD_RGB_VSYNC
#define BSP_LCD_HSYNC             BSP_LCD_RGB_HSYNC
#define BSP_LCD_DE                BSP_LCD_RGB_DE
#define BSP_LCD_PCLK              BSP_LCD_RGB_PCLK
#define BSP_LCD_DISP              BSP_LCD_RGB_DISP
#define BSP_LCD_DATA0             BSP_LCD_RGB_DATA0
#define BSP_LCD_DATA1             BSP_LCD_RGB_DATA1
#define BSP_LCD_DATA2             BSP_LCD_RGB_DATA2
#define BSP_LCD_DATA3             BSP_LCD_RGB_DATA3
#define BSP_LCD_DATA4             BSP_LCD_RGB_DATA4
#define BSP_LCD_DATA5             BSP_LCD_RGB_DATA5
#define BSP_LCD_DATA6             BSP_LCD_RGB_DATA6
#define BSP_LCD_DATA7             BSP_LCD_RGB_DATA7
#define BSP_LCD_DATA8             BSP_LCD_RGB_DATA8
#define BSP_LCD_DATA9             BSP_LCD_RGB_DATA9
#define BSP_LCD_DATA10            BSP_LCD_RGB_DATA10
#define BSP_LCD_DATA11            BSP_LCD_RGB_DATA11
#define BSP_LCD_DATA12            BSP_LCD_RGB_DATA12
#define BSP_LCD_DATA13            BSP_LCD_RGB_DATA13
#define BSP_LCD_DATA14            BSP_LCD_RGB_DATA14
#define BSP_LCD_DATA15            BSP_LCD_RGB_DATA15

/* USB uses dedicated USB_DP/USB_DM and USB1P1 pins on the ESP32-S31 module. */
#define BSP_USB_DEVICE_DP         (GPIO_NUM_NC)
#define BSP_USB_DEVICE_DM         (GPIO_NUM_NC)
#define BSP_USB_OTG_DP            (GPIO_NUM_NC)
#define BSP_USB_OTG_DM            (GPIO_NUM_NC)

/* SD card: 4-bit SDMMC */
#define BSP_SD_D0                 (GPIO_NUM_20)
#define BSP_SD_D1                 (GPIO_NUM_21)
#define BSP_SD_D2                 (GPIO_NUM_22)
#define BSP_SD_D3                 (GPIO_NUM_23)
#define BSP_SD_CLK                (GPIO_NUM_24)
#define BSP_SD_CMD                (GPIO_NUM_25)
#define BSP_SD_DET                (GPIO_NUM_NC)
#define BSP_SD_EN                 (GPIO_NUM_56)  /* SD_CTRL */
#define BSP_SD_SPI_CLK            BSP_SD_CLK
#define BSP_SD_SPI_MISO           BSP_SD_D0
#define BSP_SD_SPI_MOSI           BSP_SD_CMD
#define BSP_SD_SPI_CS             BSP_SD_D3

/* I2C peripherals */
#define BSP_RTC_I2C_ADDRESS       (PCF85063A_ADDRESS)
#define BSP_BQ27220_I2C_ADDRESS   (0x55)
#define BSP_RTC_INT               (GPIO_NUM_3)
#define BSP_IMU_I2C_ADDRESS       (0x6B)
#define BSP_IMU_INT1              (GPIO_NUM_54)
#define BSP_IMU_INT2              (GPIO_NUM_60)

/* CAN/TWAI */
#define BSP_CAN_TX                (GPIO_NUM_4)
#define BSP_CAN_RX                (GPIO_NUM_5)
#define BSP_TWAI_TX               BSP_CAN_TX
#define BSP_TWAI_RX               BSP_CAN_RX

/* RS485 */
#define BSP_RS485_TX              (GPIO_NUM_6)
#define BSP_RS485_RX              (GPIO_NUM_7)
#define BSP_RS485_EN              (GPIO_NUM_61)

/* Board LED and serial console */
#define BSP_STATUS_LED            (GPIO_NUM_55)
#define BSP_UART_TX               (GPIO_NUM_58)
#define BSP_UART_RX               (GPIO_NUM_59)
#define BSP_BOOT                  (GPIO_NUM_61)

/* Uncommitted GPIO header pins */
#define BSP_HEADER_GPIO0          (GPIO_NUM_48)
#define BSP_HEADER_GPIO1          (GPIO_NUM_49)
#define BSP_HEADER_GPIO2          (GPIO_NUM_50)
#define BSP_HEADER_GPIO3          (GPIO_NUM_51)
#define BSP_HEADER_GPIO4          (GPIO_NUM_52)
#define BSP_HEADER_GPIO5          (GPIO_NUM_53)

typedef struct {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    float temperature_c;
    uint32_t timestamp;
} bsp_imu_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * I2C
 **************************************************************************************************/

esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/**************************************************************************************************
 * Audio
 **************************************************************************************************/

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);
const audio_codec_data_if_t *bsp_audio_get_codec_itf(void);
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
esp_codec_dev_handle_t bsp_audio_codec_speaker_playback_init(void);
esp_err_t bsp_audio_codec_speaker_playback_deinit(esp_codec_dev_handle_t play_dev);
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);
esp_codec_dev_handle_t bsp_audio_codec_duplex_init(void);

/**************************************************************************************************
 * Storage
 **************************************************************************************************/

#define BSP_SPIFFS_MOUNT_POINT      CONFIG_BSP_SPIFFS_MOUNT_POINT
#define BSP_SD_MOUNT_POINT          CONFIG_BSP_SD_MOUNT_POINT
#define BSP_SDSPI_HOST              (SPI2_HOST)

typedef struct {
    const esp_vfs_fat_sdmmc_mount_config_t *mount;
    sdmmc_host_t *host;
    union {
        const sdmmc_slot_config_t *sdmmc;
        const sdspi_device_config_t *sdspi;
    } slot;
} bsp_sdcard_cfg_t;

esp_err_t bsp_spiffs_mount(void);
esp_err_t bsp_spiffs_unmount(void);
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);
sdmmc_card_t *bsp_sdcard_get_handle(void);
void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config);
void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config);
void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config);
void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config);
esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg);
esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg);

/**************************************************************************************************
 * Display
 **************************************************************************************************/

#define BSP_LCD_HSYNC_PULSE_WIDTH   (4)
#define BSP_LCD_HSYNC_BACK_PORCH    (8)
#define BSP_LCD_HSYNC_FRONT_PORCH   (8)
#define BSP_LCD_VSYNC_PULSE_WIDTH   (4)
#define BSP_LCD_VSYNC_BACK_PORCH    (8)
#define BSP_LCD_VSYNC_FRONT_PORCH   (8)
#define BSP_LCD_TOTAL_H_RES         (BSP_LCD_H_RES + BSP_LCD_HSYNC_PULSE_WIDTH + BSP_LCD_HSYNC_BACK_PORCH + BSP_LCD_HSYNC_FRONT_PORCH)
#define BSP_LCD_TOTAL_V_RES         (BSP_LCD_V_RES + BSP_LCD_VSYNC_PULSE_WIDTH + BSP_LCD_VSYNC_BACK_PORCH + BSP_LCD_VSYNC_FRONT_PORCH)
#define BSP_LCD_PIXEL_CLOCK_HZ      (BSP_LCD_TOTAL_H_RES * BSP_LCD_TOTAL_V_RES * CONFIG_BSP_LCD_REFRESH_RATE_HZ)

#define BSP_DISPLAY_PANEL_RGB_TIMING() \
    {                                  \
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ, \
        .h_res = BSP_LCD_H_RES,        \
        .v_res = BSP_LCD_V_RES,        \
        .hsync_pulse_width = BSP_LCD_HSYNC_PULSE_WIDTH, \
        .hsync_back_porch = BSP_LCD_HSYNC_BACK_PORCH, \
        .hsync_front_porch = BSP_LCD_HSYNC_FRONT_PORCH, \
        .vsync_pulse_width = BSP_LCD_VSYNC_PULSE_WIDTH, \
        .vsync_back_porch = BSP_LCD_VSYNC_BACK_PORCH, \
        .vsync_front_porch = BSP_LCD_VSYNC_FRONT_PORCH, \
        .flags.pclk_active_neg = true, \
    }

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
typedef struct {
    esp_lv_adapter_config_t lv_adapter_cfg;
    esp_lv_adapter_rotation_t rotation;
    esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode;
    uint32_t buffer_height;
    bool use_psram;
    bool enable_ppa_accel;
    bool require_double_buffer;
    struct {
        unsigned int swap_xy: 1;
        unsigned int mirror_x: 1;
        unsigned int mirror_y: 1;
    } touch_flags;
} bsp_display_cfg_t;

lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
lv_indev_t *bsp_display_get_input_dev(void);
bool bsp_display_lock(int32_t timeout_ms);
void bsp_display_unlock(void);
esp_err_t bsp_display_enter_sleep(void);
esp_err_t bsp_display_exit_sleep(void);
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);
#endif

/**************************************************************************************************
 * RTC
 **************************************************************************************************/

esp_err_t bsp_rtc_init(void);
esp_err_t bsp_set_rtc_time_date(pcf85063a_datetime_t time);
esp_err_t bsp_set_rtc_alarm_time(pcf85063a_datetime_t time);
esp_err_t bsp_get_rtc_time_date(pcf85063a_datetime_t *time);
esp_err_t bsp_get_rtc_alarm_time(pcf85063a_datetime_t *time);
esp_err_t bsp_enable_rtc_alarm(void);
esp_err_t bsp_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time);

/**************************************************************************************************
 * IMU
 **************************************************************************************************/

esp_err_t bsp_imu_init(void);
esp_err_t bsp_imu_get_who_am_i(uint8_t *who_am_i);
esp_err_t bsp_imu_is_data_ready(bool *ready);
esp_err_t bsp_imu_read(bsp_imu_data_t *data);

/**************************************************************************************************
 * Feature power/control
 **************************************************************************************************/

typedef enum {
    BSP_FEATURE_SD,
    BSP_FEATURE_CAMERA,
} bsp_feature_t;

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable);

#ifdef __cplusplus
}
#endif
