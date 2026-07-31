/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_BQ27220_CURRENT_IDLE = 0,
    BSP_BQ27220_CURRENT_CHARGING,
    BSP_BQ27220_CURRENT_DISCHARGING,
} bsp_bq27220_current_direction_t;

typedef enum {
    BSP_BQ27220_VALID_VOLTAGE = (1U << 0),
    BSP_BQ27220_VALID_CURRENT = (1U << 1),
    BSP_BQ27220_VALID_TEMPERATURE = (1U << 2),
    BSP_BQ27220_VALID_INTERNAL_TEMPERATURE = (1U << 3),
    BSP_BQ27220_VALID_STATE_OF_CHARGE = (1U << 4),
    BSP_BQ27220_VALID_STATE_OF_HEALTH = (1U << 5),
    BSP_BQ27220_VALID_REMAINING_CAPACITY = (1U << 6),
    BSP_BQ27220_VALID_FULL_CHARGE_CAPACITY = (1U << 7),
    BSP_BQ27220_VALID_DESIGN_CAPACITY = (1U << 8),
    BSP_BQ27220_VALID_TIME_TO_EMPTY = (1U << 9),
    BSP_BQ27220_VALID_TIME_TO_FULL = (1U << 10),
    BSP_BQ27220_VALID_STANDBY_CURRENT = (1U << 11),
    BSP_BQ27220_VALID_MAX_LOAD_CURRENT = (1U << 12),
    BSP_BQ27220_VALID_AVERAGE_POWER = (1U << 13),
} bsp_bq27220_valid_field_t;

typedef enum {
    BSP_BQ27220_BATTERY_STATUS_DISCHARGING = (1U << 0),
    BSP_BQ27220_BATTERY_STATUS_SHUTDOWN_REQUIRED = (1U << 1),
    BSP_BQ27220_BATTERY_STATUS_DISCHARGE_TERMINATION = (1U << 2),
    BSP_BQ27220_BATTERY_STATUS_PRESENT = (1U << 3),
    BSP_BQ27220_BATTERY_STATUS_INSERTED = (1U << 4),
    BSP_BQ27220_BATTERY_STATUS_GOOD_OCV = (1U << 5),
    BSP_BQ27220_BATTERY_STATUS_CHARGE_TERMINATION = (1U << 6),
    BSP_BQ27220_BATTERY_STATUS_CHARGE_INHIBITED = (1U << 8),
    BSP_BQ27220_BATTERY_STATUS_FULL_CHARGE = (1U << 9),
    BSP_BQ27220_BATTERY_STATUS_OVER_TEMP_DISCHARGE = (1U << 10),
    BSP_BQ27220_BATTERY_STATUS_OVER_TEMP_CHARGE = (1U << 11),
    BSP_BQ27220_BATTERY_STATUS_SLEEP = (1U << 12),
    BSP_BQ27220_BATTERY_STATUS_OCV_READ_FAILED = (1U << 13),
    BSP_BQ27220_BATTERY_STATUS_OCV_UPDATE_COMPLETE = (1U << 14),
    BSP_BQ27220_BATTERY_STATUS_FULL_DISCHARGE = (1U << 15),
} bsp_bq27220_battery_status_flag_t;

typedef enum {
    BSP_BQ27220_OPERATION_STATUS_CALIBRATION_MODE = (1U << 0),
    BSP_BQ27220_OPERATION_STATUS_EDV2 = (1U << 3),
    BSP_BQ27220_OPERATION_STATUS_DISCHARGE_QUALIFIED = (1U << 4),
    BSP_BQ27220_OPERATION_STATUS_INIT_COMPLETE = (1U << 5),
    BSP_BQ27220_OPERATION_STATUS_CAPACITY_THROTTLED = (1U << 6),
    BSP_BQ27220_OPERATION_STATUS_BTP_THRESHOLD = (1U << 7),
    BSP_BQ27220_OPERATION_STATUS_CONFIG_UPDATE = (1U << 10),
} bsp_bq27220_operation_status_flag_t;

typedef struct {
    uint16_t device_id;
    uint32_t firmware_version;
    uint64_t sample_time_ms;
    uint32_t valid_fields;
    uint16_t battery_status_flags;
    uint16_t operation_status_flags;
    uint8_t security_access_level;
    bool battery_present;
    bsp_bq27220_current_direction_t current_direction;

    uint16_t voltage_mv;
    int16_t current_ma;
    int16_t standby_current_ma;
    int16_t max_load_current_ma;
    int16_t average_power_mw;
    float temperature_c;
    float internal_temperature_c;
    uint8_t state_of_charge_percent;
    uint8_t state_of_health_percent;
    uint16_t remaining_capacity_mah;
    uint16_t full_charge_capacity_mah;
    uint16_t design_capacity_mah;
    uint16_t time_to_empty_min;
    uint16_t time_to_full_min;
    uint16_t cycle_count;
} bsp_bq27220_snapshot_t;

/** Initialize and identify the BQ27220 on the BSP shared I2C bus. Idempotent. */
esp_err_t bsp_bq27220_init(void);

/** Return true after a successful BQ27220 initialization. */
bool bsp_bq27220_is_available(void);

/**
 * Provision the rated capacity of a new battery in mAh.
 *
 * This follows the GaugeBQ27220 reference flow and initializes both Design
 * Capacity and Full Charge Capacity. The persistent configuration is written
 * only when Design Capacity differs, so repeated calls do not overwrite a
 * learned Full Charge Capacity. Call from a worker/startup context, not from
 * the LVGL thread. A value of zero is invalid. Cell-specific CEDV coefficients
 * and voltage/current thresholds are intentionally left unchanged.
 */
esp_err_t bsp_bq27220_configure_capacity(uint16_t capacity_mah);

/** Read one coherent SensorLib snapshot. Invalid optional fields are cleared in valid_fields. */
esp_err_t bsp_bq27220_read_snapshot(bsp_bq27220_snapshot_t *snapshot);

/** Release only the BQ27220 device. The BSP shared I2C bus remains initialized. */
esp_err_t bsp_bq27220_deinit(void);

#ifdef __cplusplus
}
#endif
