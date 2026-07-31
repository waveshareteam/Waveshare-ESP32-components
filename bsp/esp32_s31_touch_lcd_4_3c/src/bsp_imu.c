/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "qmi8658.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"

static const char *TAG = "bsp_imu";

static qmi8658_dev_t s_qmi8658_dev;
static bool s_qmi8658_initialized;

esp_err_t bsp_imu_init(void)
{
    if (s_qmi8658_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "I2C init failed");
    ESP_RETURN_ON_ERROR(qmi8658_init(&s_qmi8658_dev, bsp_i2c_get_handle(), BSP_IMU_I2C_ADDRESS),
                        TAG, "QMI8658 init failed");

    qmi8658_set_accel_unit_mps2(&s_qmi8658_dev, true);
    qmi8658_set_gyro_unit_rads(&s_qmi8658_dev, true);

    s_qmi8658_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_imu_get_who_am_i(uint8_t *who_am_i)
{
    ESP_RETURN_ON_FALSE(who_am_i, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(bsp_imu_init(), TAG, "IMU init failed");

    return qmi8658_get_who_am_i(&s_qmi8658_dev, who_am_i);
}

esp_err_t bsp_imu_is_data_ready(bool *ready)
{
    ESP_RETURN_ON_FALSE(ready, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(bsp_imu_init(), TAG, "IMU init failed");

    return qmi8658_is_data_ready(&s_qmi8658_dev, ready);
}

esp_err_t bsp_imu_read(bsp_imu_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(bsp_imu_init(), TAG, "IMU init failed");

    qmi8658_data_t qmi_data = {0};
    ESP_RETURN_ON_ERROR(qmi8658_read_sensor_data(&s_qmi8658_dev, &qmi_data), TAG, "read sensor data failed");

    data->accel_x_mps2 = qmi_data.accelX;
    data->accel_y_mps2 = qmi_data.accelY;
    data->accel_z_mps2 = qmi_data.accelZ;
    data->gyro_x_rads = qmi_data.gyroX;
    data->gyro_y_rads = qmi_data.gyroY;
    data->gyro_z_rads = qmi_data.gyroZ;
    data->temperature_c = qmi_data.temperature;
    data->timestamp = qmi_data.timestamp;

    return ESP_OK;
}
