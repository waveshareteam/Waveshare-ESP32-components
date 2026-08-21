/*
 * SPDX-FileCopyrightText: 2026 Waveshare Team
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdspi_host.h"
#include "esp_codec_dev.h"
#include "sdmmc_cmd.h"

#include "bsp/config.h"
#include "bsp/display.h"
#include "bsp/touch.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "esp_lv_adapter.h"
#include "lvgl.h"
#endif

/* BSP capabilities */
#define BSP_CAPS_DISPLAY       1
#define BSP_CAPS_TOUCH         1
#define BSP_CAPS_BUTTONS       0
#define BSP_CAPS_AUDIO         1
#define BSP_CAPS_AUDIO_SPEAKER 1
#define BSP_CAPS_AUDIO_MIC     1
#define BSP_CAPS_SDCARD        1
#define BSP_CAPS_IMU           1
#define BSP_CAPS_RTC           1
#define BSP_CAPS_PMU           1

/* Shared I2C0 bus */
#define BSP_I2C_NUM (I2C_NUM_0)
#define BSP_I2C_SCL (GPIO_NUM_7)
#define BSP_I2C_SDA (GPIO_NUM_8)

/* I2S0 audio bus */
#define BSP_I2S_NUM  (I2S_NUM_0)
#define BSP_I2S_MCLK (GPIO_NUM_19)
#define BSP_I2S_SCLK (GPIO_NUM_20)
#define BSP_I2S_DSIN (GPIO_NUM_21)
#define BSP_I2S_LCLK (GPIO_NUM_22)
#define BSP_I2S_DOUT (GPIO_NUM_23)

/* 480 x 480 SH8601 QSPI AMOLED */
#define BSP_LCD_SPI_NUM   (SPI2_HOST)
#define BSP_LCD_PCLK      (GPIO_NUM_0)
#define BSP_LCD_DATA0     (GPIO_NUM_1)
#define BSP_LCD_DATA1     (GPIO_NUM_2)
#define BSP_LCD_DATA2     (GPIO_NUM_3)
#define BSP_LCD_DATA3     (GPIO_NUM_4)
#define BSP_LCD_CS        (GPIO_NUM_15)
#define BSP_LCD_RST       (GPIO_NUM_NC) /* AXP2101 ALDO3 */
#define BSP_LCD_BACKLIGHT (GPIO_NUM_NC) /* SH8601 command 0x51 */

/* CST9217 touch */
#define BSP_LCD_TOUCH_RST (GPIO_NUM_11)
#define BSP_LCD_TOUCH_INT (GPIO_NUM_5)

/* SDSPI shares SPI2 clock/data0/data1 with the QSPI display. */
#define BSP_SD_SCK  (GPIO_NUM_0)
#define BSP_SD_MOSI (GPIO_NUM_1)
#define BSP_SD_MISO (GPIO_NUM_2)
#define BSP_SD_SDCS (GPIO_NUM_6)

/* Shared I2C device addresses and interrupt pins */
#define BSP_PMU_I2C_ADDRESS     (0x34)
#define BSP_IMU_I2C_ADDRESS     (0x6B)
#define BSP_IMU_INT1            (GPIO_NUM_16)
#define BSP_IMU_INT2            (GPIO_NUM_17)
#define BSP_RTC_I2C_ADDRESS     (0x51)
#define BSP_RTC_INT             (GPIO_NUM_18)
#define BSP_TOUCH_I2C_ADDRESS   (0x5A)
#define BSP_AUDIO_OUT_I2C_ADDR  (0x18)
#define BSP_AUDIO_IN_I2C_ADDR   (0x40)

#define BSP_SPIFFS_MOUNT_POINT CONFIG_BSP_SPIFFS_MOUNT_POINT
#define BSP_SD_MOUNT_POINT     CONFIG_BSP_SD_MOUNT_POINT

#ifdef __cplusplus
extern "C" {
#endif

/* I2C */
esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/* AXP2101 PMU */
typedef enum {
    BSP_PMU_CHARGE_TRI_STATE = 0,
    BSP_PMU_CHARGE_PRECHARGE = 1,
    BSP_PMU_CHARGE_CONSTANT_CURRENT = 2,
    BSP_PMU_CHARGE_CONSTANT_VOLTAGE = 3,
    BSP_PMU_CHARGE_DONE = 4,
    BSP_PMU_CHARGE_STOPPED = 5,
} bsp_pmu_charger_status_t;

typedef struct {
    bool battery_present;
    bool charging;
    bsp_pmu_charger_status_t charger_status;
    uint16_t battery_mv;
} bsp_pmu_snapshot_t;

/** Initialize AXP2101 and apply the product rail/charger configuration. */
esp_err_t bsp_pmu_init(void);
esp_err_t bsp_pmu_get_snapshot(bsp_pmu_snapshot_t *snapshot);
const char *bsp_pmu_charger_status_to_string(bsp_pmu_charger_status_t status);
esp_err_t bsp_pmu_set_display_power(bool enable);
esp_err_t bsp_pmu_set_audio_power(bool enable);

/* Filesystems */
esp_err_t bsp_spiffs_mount(void);
esp_err_t bsp_spiffs_unmount(void);

extern sdmmc_card_t *bsp_sdcard;
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);

/* ES8311 playback + ES7210 capture */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);
esp_err_t bsp_audio_poweramp_enable(bool enable);

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
typedef struct {
    esp_lv_adapter_config_t lv_adapter_cfg;
    esp_lv_adapter_rotation_t rotation;
    esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode;
    bsp_touch_config_t touch;
} bsp_display_cfg_t;

lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(bsp_display_cfg_t *cfg);
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take the LVGL adapter mutex.
 *
 * @param timeout_ms Timeout in milliseconds. Use 0 to wait indefinitely.
 * @return true when the mutex was acquired, otherwise false.
 */
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);
#endif

#ifdef __cplusplus
}
#endif
