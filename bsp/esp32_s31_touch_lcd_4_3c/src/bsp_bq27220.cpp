/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/bq27220.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <inttypes.h>
#include <new>

#include "GaugeBQ27220.hpp"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "bsp_bq27220";
constexpr uint32_t I2C_TIMEOUT_MS = 100;
constexpr uint16_t BQ27220_DEVICE_ID = 0x0220;
constexpr uint16_t INVALID_WORD = UINT16_MAX;
constexpr uint32_t CONFIG_REINIT_DELAY_MS = 1000;
constexpr float MIN_VALID_TEMPERATURE_C = -50.0f;
constexpr float MAX_VALID_TEMPERATURE_C = 150.0f;

GaugeBQ27220 s_gauge;
i2c_master_dev_handle_t s_i2c_device;
bool s_initialized;
uint16_t s_device_id;
uint8_t s_pending_read_register;
bool s_pending_read_register_valid;
uint32_t s_firmware_version;
esp_err_t s_transport_error = ESP_OK;

SemaphoreHandle_t get_mutex()
{
    static StaticSemaphore_t storage;
    static SemaphoreHandle_t mutex = xSemaphoreCreateMutexStatic(&storage);
    return mutex;
}

class LockGuard {
public:
    explicit LockGuard(SemaphoreHandle_t mutex) : mutex_(mutex)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }

    ~LockGuard()
    {
        xSemaphoreGive(mutex_);
    }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

private:
    SemaphoreHandle_t mutex_;
};

bool sensor_comm_callback(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length,
                          bool write_register, bool is_write)
{
    if (!s_i2c_device || address != BSP_BQ27220_I2C_ADDRESS) {
        s_pending_read_register_valid = false;
        s_transport_error = ESP_ERR_INVALID_STATE;
        return false;
    }
    if (!buffer || length == 0) {
        s_pending_read_register_valid = false;
        s_transport_error = ESP_ERR_INVALID_ARG;
        return false;
    }

    if (is_write) {
        if (write_register) {
            s_pending_read_register_valid = false;
            i2c_master_transmit_multi_buffer_info_t buffers[2] = {};
            buffers[0].write_buffer = &reg;
            buffers[0].buffer_size = sizeof(reg);
            buffers[1].write_buffer = buffer;
            buffers[1].buffer_size = length;
            s_transport_error = i2c_master_multi_buffer_transmit(
                s_i2c_device, buffers, 2, I2C_TIMEOUT_MS);
            return s_transport_error == ESP_OK;
        }

        // SensorCommCustom implements writeThenRead as two callbacks. Defer a
        // one-byte register pointer so the following read stays atomic on I2C.
        if (length == 1) {
            s_pending_read_register = buffer[0];
            s_pending_read_register_valid = true;
            s_transport_error = ESP_OK;
            return true;
        }

        s_pending_read_register_valid = false;
        s_transport_error = i2c_master_transmit(s_i2c_device, buffer, length, I2C_TIMEOUT_MS);
        return s_transport_error == ESP_OK;
    }

    if (write_register) {
        s_pending_read_register_valid = false;
        s_transport_error = i2c_master_transmit_receive(
            s_i2c_device, &reg, sizeof(reg), buffer, length, I2C_TIMEOUT_MS);
    } else if (s_pending_read_register_valid) {
        const uint8_t pending_register = s_pending_read_register;
        s_pending_read_register_valid = false;
        s_transport_error = i2c_master_transmit_receive(
            s_i2c_device, &pending_register, sizeof(pending_register), buffer, length,
            I2C_TIMEOUT_MS);
    } else {
        s_transport_error = i2c_master_receive(s_i2c_device, buffer, length, I2C_TIMEOUT_MS);
    }
    return s_transport_error == ESP_OK;
}

uint32_t sensor_hal_callback(SensorCommCustomHal::Operation operation, void *param1, void *)
{
    const uint32_t value = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(param1));
    switch (operation) {
    case SensorCommCustomHal::OP_MILLIS:
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    case SensorCommCustomHal::OP_DELAY: {
        const TickType_t ticks = pdMS_TO_TICKS(value);
        if (ticks > 0) {
            vTaskDelay(ticks);
        } else if (value > 0) {
            esp_rom_delay_us(value * 1000U);
        }
        break;
    }
    case SensorCommCustomHal::OP_DELAYMICROSECONDS:
        esp_rom_delay_us(value);
        break;
    default:
        break;
    }
    return 0;
}

void reset_driver()
{
    s_gauge.~GaugeBQ27220();
    new (&s_gauge) GaugeBQ27220();
    s_initialized = false;
    s_device_id = 0;
    s_firmware_version = 0;
    s_pending_read_register_valid = false;
    s_transport_error = ESP_OK;
}

esp_err_t remove_i2c_device()
{
    if (!s_i2c_device) {
        return ESP_OK;
    }
    const esp_err_t error = i2c_master_bus_rm_device(s_i2c_device);
    if (error == ESP_OK) {
        s_i2c_device = nullptr;
    }
    return error;
}

esp_err_t init_locked()
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t error = remove_i2c_device();
    if (error != ESP_OK) {
        return error;
    }

    error = bsp_i2c_init();
    if (error != ESP_OK) {
        return error;
    }
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    error = i2c_master_probe(bus, BSP_BQ27220_I2C_ADDRESS, I2C_TIMEOUT_MS);
    if (error != ESP_OK) {
        return error;
    }

    i2c_device_config_t config = {};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = BSP_BQ27220_I2C_ADDRESS;
    config.scl_speed_hz = 400000;
    error = i2c_master_bus_add_device(bus, &config, &s_i2c_device);
    if (error != ESP_OK) {
        return error;
    }

    s_transport_error = ESP_OK;
    if (!s_gauge.begin(sensor_comm_callback, sensor_hal_callback)) {
        error = (s_transport_error == ESP_OK) ? ESP_ERR_NOT_FOUND : s_transport_error;
        reset_driver();
        remove_i2c_device();
        return error;
    }

    s_transport_error = ESP_OK;
    const uint16_t device_id = s_gauge.getChipID();
    if (device_id != BQ27220_DEVICE_ID) {
        reset_driver();
        remove_i2c_device();
        return ESP_ERR_NOT_FOUND;
    }
    const int firmware_version = s_gauge.getFirmwareVersion();
    if (firmware_version < 0) {
        error = (s_transport_error == ESP_OK) ? ESP_FAIL : s_transport_error;
        reset_driver();
        remove_i2c_device();
        return error;
    }

    s_device_id = device_id;
    s_firmware_version = static_cast<uint32_t>(firmware_version);
    s_transport_error = ESP_OK;
    s_initialized = true;
    ESP_LOGI(TAG, "BQ27220 detected (device=0x%04x, firmware=0x%08" PRIx32 ")",
             s_device_id, s_firmware_version);
    return ESP_OK;
}

bool valid_temperature(float value)
{
    return std::isfinite(value) && value >= MIN_VALID_TEMPERATURE_C &&
        value <= MAX_VALID_TEMPERATURE_C;
}

uint16_t get_battery_status_flags(const BatteryStatus &status)
{
    uint16_t flags = 0;
    if (status.isInDischargeMode()) flags |= BSP_BQ27220_BATTERY_STATUS_DISCHARGING;
    if (status.isSystemShutdownRequired()) flags |= BSP_BQ27220_BATTERY_STATUS_SHUTDOWN_REQUIRED;
    if (status.isDischargeTerminationAlarm()) flags |= BSP_BQ27220_BATTERY_STATUS_DISCHARGE_TERMINATION;
    if (status.isBatteryPresent()) flags |= BSP_BQ27220_BATTERY_STATUS_PRESENT;
    if (status.isBatteryInserted()) flags |= BSP_BQ27220_BATTERY_STATUS_INSERTED;
    if (status.isGoodOcvMeasurement()) flags |= BSP_BQ27220_BATTERY_STATUS_GOOD_OCV;
    if (status.isChargingTerminationAlarm()) flags |= BSP_BQ27220_BATTERY_STATUS_CHARGE_TERMINATION;
    if (status.isChargeInhibited()) flags |= BSP_BQ27220_BATTERY_STATUS_CHARGE_INHIBITED;
    if (status.isFullChargeDetected()) flags |= BSP_BQ27220_BATTERY_STATUS_FULL_CHARGE;
    if (status.isOverTemperatureDuringDischarge()) flags |= BSP_BQ27220_BATTERY_STATUS_OVER_TEMP_DISCHARGE;
    if (status.isOverTemperatureDuringCharging()) flags |= BSP_BQ27220_BATTERY_STATUS_OVER_TEMP_CHARGE;
    if (status.isInSleepMode()) flags |= BSP_BQ27220_BATTERY_STATUS_SLEEP;
    if (status.isOcvReadFailedDueToCurrent()) flags |= BSP_BQ27220_BATTERY_STATUS_OCV_READ_FAILED;
    if (status.isOcvMeasurementUpdateComplete()) flags |= BSP_BQ27220_BATTERY_STATUS_OCV_UPDATE_COMPLETE;
    if (status.isFullDischargeDetected()) flags |= BSP_BQ27220_BATTERY_STATUS_FULL_DISCHARGE;
    return flags;
}

uint16_t get_operation_status_flags(const FuelGaugeOperationStatus &status)
{
    uint16_t flags = 0;
    if (status.getIsCalibrationModeEnabled()) flags |= BSP_BQ27220_OPERATION_STATUS_CALIBRATION_MODE;
    if (status.getIsBatteryVoltageBelowEdv2()) flags |= BSP_BQ27220_OPERATION_STATUS_EDV2;
    if (status.getIsDischargeCycleCompliant()) flags |= BSP_BQ27220_OPERATION_STATUS_DISCHARGE_QUALIFIED;
    if (status.getIsInitializationComplete()) flags |= BSP_BQ27220_OPERATION_STATUS_INIT_COMPLETE;
    if (status.getIsCapacityAccumulationThrottled()) flags |= BSP_BQ27220_OPERATION_STATUS_CAPACITY_THROTTLED;
    if (status.getIsBtpThresholdExceeded()) flags |= BSP_BQ27220_OPERATION_STATUS_BTP_THRESHOLD;
    if (status.getIsConfigUpdateMode()) flags |= BSP_BQ27220_OPERATION_STATUS_CONFIG_UPDATE;
    return flags;
}

} // namespace

extern "C" esp_err_t bsp_bq27220_init(void)
{
    LockGuard lock(get_mutex());
    return init_locked();
}

extern "C" bool bsp_bq27220_is_available(void)
{
    LockGuard lock(get_mutex());
    return s_initialized;
}

extern "C" esp_err_t bsp_bq27220_configure_capacity(uint16_t capacity_mah)
{
    if (capacity_mah == 0 || capacity_mah == INVALID_WORD) {
        return ESP_ERR_INVALID_ARG;
    }

    LockGuard lock(get_mutex());
    esp_err_t error = init_locked();
    if (error != ESP_OK) {
        return error;
    }
    s_transport_error = ESP_OK;
    if (!s_gauge.refresh()) {
        return (s_transport_error == ESP_OK) ? ESP_FAIL : s_transport_error;
    }

    if (s_gauge.getDesignCapacity() == capacity_mah) {
        ESP_LOGI(TAG, "Design capacity is already %u mAh; retaining learned full capacity",
                 static_cast<unsigned>(capacity_mah));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Provisioning Design/Full Charge Capacity to %u mAh",
             static_cast<unsigned>(capacity_mah));
    s_transport_error = ESP_OK;
    if (!s_gauge.setNewCapacity(capacity_mah, capacity_mah)) {
        ESP_LOGE(TAG, "GaugeBQ27220 capacity configuration failed");
        return (s_transport_error == ESP_OK) ? ESP_FAIL : s_transport_error;
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_REINIT_DELAY_MS));
    s_transport_error = ESP_OK;
    if (!s_gauge.refresh()) {
        return (s_transport_error == ESP_OK) ? ESP_FAIL : s_transport_error;
    }
    if (s_gauge.getDesignCapacity() != capacity_mah ||
            s_gauge.getFullChargeCapacity() != capacity_mah) {
        ESP_LOGE(TAG, "Capacity readback mismatch (design=%u, full=%u, expected=%u)",
                 static_cast<unsigned>(s_gauge.getDesignCapacity()),
                 static_cast<unsigned>(s_gauge.getFullChargeCapacity()),
                 static_cast<unsigned>(capacity_mah));
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "BQ27220 capacity configuration verified");
    return ESP_OK;
}

extern "C" esp_err_t bsp_bq27220_read_snapshot(bsp_bq27220_snapshot_t *snapshot)
{
    if (!snapshot) {
        return ESP_ERR_INVALID_ARG;
    }

    LockGuard lock(get_mutex());
    esp_err_t error = init_locked();
    if (error != ESP_OK) {
        std::memset(snapshot, 0, sizeof(*snapshot));
        return error;
    }

    s_transport_error = ESP_OK;
    if (!s_gauge.refresh()) {
        std::memset(snapshot, 0, sizeof(*snapshot));
        return (s_transport_error == ESP_OK) ? ESP_FAIL : s_transport_error;
    }

    bsp_bq27220_snapshot_t value = {};
    value.device_id = s_device_id;
    value.firmware_version = s_firmware_version;
    value.sample_time_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);

    const BatteryStatus battery_status = s_gauge.getBatteryStatus();
    const FuelGaugeOperationStatus operation_status = s_gauge.getOperationStatus();
    value.battery_status_flags = get_battery_status_flags(battery_status);
    value.operation_status_flags = get_operation_status_flags(operation_status);
    value.security_access_level = operation_status.getSecurityAccessLevel();
    value.battery_present = battery_status.isBatteryPresent();

    value.voltage_mv = s_gauge.getVoltage();
    if (value.voltage_mv > 0 && value.voltage_mv <= 6000) {
        value.valid_fields |= BSP_BQ27220_VALID_VOLTAGE;
    }

    value.current_ma = s_gauge.getCurrent();
    value.valid_fields |= BSP_BQ27220_VALID_CURRENT;
    if (value.current_ma > 0) {
        value.current_direction = BSP_BQ27220_CURRENT_CHARGING;
    } else if (value.current_ma < 0 || battery_status.isInDischargeMode()) {
        value.current_direction = BSP_BQ27220_CURRENT_DISCHARGING;
    }

    value.standby_current_ma = s_gauge.getStandbyCurrent();
    value.valid_fields |= BSP_BQ27220_VALID_STANDBY_CURRENT;
    value.max_load_current_ma = s_gauge.getMaxLoadCurrent();
    value.valid_fields |= BSP_BQ27220_VALID_MAX_LOAD_CURRENT;
    value.average_power_mw = s_gauge.getAveragePower();
    value.valid_fields |= BSP_BQ27220_VALID_AVERAGE_POWER;

    value.temperature_c = s_gauge.getTemperature();
    if (valid_temperature(value.temperature_c)) {
        value.valid_fields |= BSP_BQ27220_VALID_TEMPERATURE;
    }
    value.internal_temperature_c = s_gauge.getInternalTemperature();
    if (valid_temperature(value.internal_temperature_c)) {
        value.valid_fields |= BSP_BQ27220_VALID_INTERNAL_TEMPERATURE;
    }

    const uint16_t state_of_charge = s_gauge.getStateOfCharge();
    if (state_of_charge <= 100) {
        value.state_of_charge_percent = static_cast<uint8_t>(state_of_charge);
        value.valid_fields |= BSP_BQ27220_VALID_STATE_OF_CHARGE;
    }
    const uint16_t state_of_health = s_gauge.getStateOfHealth();
    if (state_of_health <= 100) {
        value.state_of_health_percent = static_cast<uint8_t>(state_of_health);
        value.valid_fields |= BSP_BQ27220_VALID_STATE_OF_HEALTH;
    }

    value.remaining_capacity_mah = s_gauge.getRemainingCapacity();
    if (value.remaining_capacity_mah != INVALID_WORD) {
        value.valid_fields |= BSP_BQ27220_VALID_REMAINING_CAPACITY;
    }
    value.full_charge_capacity_mah = s_gauge.getFullChargeCapacity();
    if (value.full_charge_capacity_mah != INVALID_WORD) {
        value.valid_fields |= BSP_BQ27220_VALID_FULL_CHARGE_CAPACITY;
    }
    value.design_capacity_mah = s_gauge.getDesignCapacity();
    if (value.design_capacity_mah > 0 && value.design_capacity_mah != INVALID_WORD) {
        value.valid_fields |= BSP_BQ27220_VALID_DESIGN_CAPACITY;
    }

    value.time_to_empty_min = s_gauge.getTimeToEmpty();
    if (value.time_to_empty_min != INVALID_WORD) {
        value.valid_fields |= BSP_BQ27220_VALID_TIME_TO_EMPTY;
    }
    value.time_to_full_min = s_gauge.getTimeToFull();
    if (value.time_to_full_min != INVALID_WORD) {
        value.valid_fields |= BSP_BQ27220_VALID_TIME_TO_FULL;
    }
    value.cycle_count = s_gauge.getCycleCount();

    *snapshot = value;
    return ESP_OK;
}

extern "C" esp_err_t bsp_bq27220_deinit(void)
{
    LockGuard lock(get_mutex());
    reset_driver();
    return remove_i2c_device();
}
