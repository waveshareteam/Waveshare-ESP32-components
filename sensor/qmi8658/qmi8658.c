#include "qmi8658.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "QMI8658";

esp_err_t qmi8658_init(qmi8658_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr) {
    if (!dev || !bus_handle) return ESP_ERR_INVALID_ARG;
    
    dev->bus_handle = bus_handle;
    dev->accel_lsb_div = 4096;
    dev->gyro_lsb_div = 64;
    dev->accel_unit_mps2 = false;
    dev->gyro_unit_rads = false;
    dev->display_precision = 6;
    dev->timestamp = 0;
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ret;
    }
    
    uint8_t who_am_i;
    ret = qmi8658_get_who_am_i(dev, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    
    if (who_am_i != 0x05) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value: 0x%02X, expected 0x05", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    
    ret = qmi8658_write_register(dev, QMI8658_CTRL1, 0x60);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        return ret;
    }
    
    ret = qmi8658_set_accel_range(dev, QMI8658_ACCEL_RANGE_8G);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_set_accel_odr(dev, QMI8658_ACCEL_ODR_1000HZ);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_set_gyro_range(dev, QMI8658_GYRO_RANGE_512DPS);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_set_gyro_odr(dev, QMI8658_GYRO_ODR_1000HZ);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_enable_sensors(dev, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    
    ESP_LOGI(TAG, "QMI8658 initialized successfully");
    return ret;
}

esp_err_t qmi8658_set_accel_range(qmi8658_dev_t *dev, qmi8658_accel_range_t range) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    switch (range) {
        case QMI8658_ACCEL_RANGE_2G:
            dev->accel_lsb_div = 16384;
            break;
        case QMI8658_ACCEL_RANGE_4G:
            dev->accel_lsb_div = 8192;
            break;
        case QMI8658_ACCEL_RANGE_8G:
            dev->accel_lsb_div = 4096;
            break;
        case QMI8658_ACCEL_RANGE_16G:
            dev->accel_lsb_div = 2048;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return qmi8658_write_register(dev, QMI8658_CTRL2, (range << 4) | 0x03);
}

esp_err_t qmi8658_set_accel_odr(qmi8658_dev_t *dev, qmi8658_accel_odr_t odr) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    uint8_t current_ctrl2;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL2, &current_ctrl2, 1);
    if (ret != ESP_OK) return ret;
    
    uint8_t new_ctrl2 = (current_ctrl2 & 0xF0) | odr;
    return qmi8658_write_register(dev, QMI8658_CTRL2, new_ctrl2);
}

esp_err_t qmi8658_set_gyro_range(qmi8658_dev_t *dev, qmi8658_gyro_range_t range) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    switch (range) {
        case QMI8658_GYRO_RANGE_32DPS:
            dev->gyro_lsb_div = 1024;
            break;
        case QMI8658_GYRO_RANGE_64DPS:
            dev->gyro_lsb_div = 512;
            break;
        case QMI8658_GYRO_RANGE_128DPS:
            dev->gyro_lsb_div = 256;
            break;
        case QMI8658_GYRO_RANGE_256DPS:
            dev->gyro_lsb_div = 128;
            break;
        case QMI8658_GYRO_RANGE_512DPS:
            dev->gyro_lsb_div = 64;
            break;
        case QMI8658_GYRO_RANGE_1024DPS:
            dev->gyro_lsb_div = 32;
            break;
        case QMI8658_GYRO_RANGE_2048DPS:
            dev->gyro_lsb_div = 16;
            break;
        case QMI8658_GYRO_RANGE_4096DPS:
            dev->gyro_lsb_div = 8;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return qmi8658_write_register(dev, QMI8658_CTRL3, (range << 4) | 0x03);
}

esp_err_t qmi8658_set_gyro_odr(qmi8658_dev_t *dev, qmi8658_gyro_odr_t odr) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    uint8_t current_ctrl3;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL3, &current_ctrl3, 1);
    if (ret != ESP_OK) return ret;
    
    uint8_t new_ctrl3 = (current_ctrl3 & 0xF0) | odr;
    return qmi8658_write_register(dev, QMI8658_CTRL3, new_ctrl3);
}

esp_err_t qmi8658_enable_accel(qmi8658_dev_t *dev, bool enable) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    uint8_t current_ctrl7;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL7, &current_ctrl7, 1);
    if (ret != ESP_OK) return ret;
    
    if (enable) {
        current_ctrl7 |= QMI8658_ENABLE_ACCEL;
    } else {
        current_ctrl7 &= ~QMI8658_ENABLE_ACCEL;
    }
    
    return qmi8658_write_register(dev, QMI8658_CTRL7, current_ctrl7);
}

esp_err_t qmi8658_enable_gyro(qmi8658_dev_t *dev, bool enable) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    uint8_t current_ctrl7;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_CTRL7, &current_ctrl7, 1);
    if (ret != ESP_OK) return ret;
    
    if (enable) {
        current_ctrl7 |= QMI8658_ENABLE_GYRO;
    } else {
        current_ctrl7 &= ~QMI8658_ENABLE_GYRO;
    }
    
    return qmi8658_write_register(dev, QMI8658_CTRL7, current_ctrl7);
}

esp_err_t qmi8658_enable_sensors(qmi8658_dev_t *dev, uint8_t enable_flags) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return qmi8658_write_register(dev, QMI8658_CTRL7, enable_flags & 0x0F);
}

esp_err_t qmi8658_read_accel(qmi8658_dev_t *dev, float *x, float *y, float *z) {
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;
    
    uint8_t buffer[6];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_AX_L, buffer, 6);
    if (ret != ESP_OK) return ret;
    
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    if (dev->accel_unit_mps2) {
        *x = (raw_x * ONE_G) / dev->accel_lsb_div;
        *y = (raw_y * ONE_G) / dev->accel_lsb_div;
        *z = (raw_z * ONE_G) / dev->accel_lsb_div;
    } else {
        *x = (raw_x * 1000.0f) / dev->accel_lsb_div;
        *y = (raw_y * 1000.0f) / dev->accel_lsb_div;
        *z = (raw_z * 1000.0f) / dev->accel_lsb_div;
    }
    
    return ESP_OK;
}

esp_err_t qmi8658_read_gyro(qmi8658_dev_t *dev, float *x, float *y, float *z) {
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;
    
    uint8_t buffer[6];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_GX_L, buffer, 6);
    if (ret != ESP_OK) return ret;
    
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    if (dev->gyro_unit_rads) {
        *x = (raw_x * M_PI / 180.0f) / dev->gyro_lsb_div;
        *y = (raw_y * M_PI / 180.0f) / dev->gyro_lsb_div;
        *z = (raw_z * M_PI / 180.0f) / dev->gyro_lsb_div;
    } else {
        *x = (float)raw_x / dev->gyro_lsb_div;
        *y = (float)raw_y / dev->gyro_lsb_div;
        *z = (float)raw_z / dev->gyro_lsb_div;
    }
    
    return ESP_OK;
}

esp_err_t qmi8658_read_temp(qmi8658_dev_t *dev, float *temperature) {
    if (!dev || !temperature) return ESP_ERR_INVALID_ARG;
    
    uint8_t buffer[2];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_TEMP_L, buffer, 2);
    if (ret != ESP_OK) return ret;
    
    int16_t raw_temp = (int16_t)((buffer[1] << 8) | buffer[0]);
    *temperature = (float)raw_temp / 256.0f;
    
    return ESP_OK;
}

esp_err_t qmi8658_read_sensor_data(qmi8658_dev_t *dev, qmi8658_data_t *data) {
    if (!dev || !data) return ESP_ERR_INVALID_ARG;
    
    uint8_t timestamp_buffer[3];
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_TIMESTAMP_L, timestamp_buffer, 3);
    if (ret == ESP_OK) {
        uint32_t timestamp = ((uint32_t)timestamp_buffer[2] << 16) | 
                           ((uint32_t)timestamp_buffer[1] << 8) | 
                           timestamp_buffer[0];
        if (timestamp > dev->timestamp) {
            dev->timestamp = timestamp;
        } else {
            dev->timestamp = (timestamp + 0x1000000 - dev->timestamp);
        }
        data->timestamp = dev->timestamp;
    }
    
    uint8_t sensor_buffer[12];
    ret = qmi8658_read_register(dev, QMI8658_AX_L, sensor_buffer, 12);
    if (ret != ESP_OK) return ret;
    
    int16_t raw_ax = (int16_t)((sensor_buffer[1] << 8) | sensor_buffer[0]);
    int16_t raw_ay = (int16_t)((sensor_buffer[3] << 8) | sensor_buffer[2]);
    int16_t raw_az = (int16_t)((sensor_buffer[5] << 8) | sensor_buffer[4]);
    
    int16_t raw_gx = (int16_t)((sensor_buffer[7] << 8) | sensor_buffer[6]);
    int16_t raw_gy = (int16_t)((sensor_buffer[9] << 8) | sensor_buffer[8]);
    int16_t raw_gz = (int16_t)((sensor_buffer[11] << 8) | sensor_buffer[10]);
    
    if (dev->accel_unit_mps2) {
        data->accelX = (raw_ax * ONE_G) / dev->accel_lsb_div;
        data->accelY = (raw_ay * ONE_G) / dev->accel_lsb_div;
        data->accelZ = (raw_az * ONE_G) / dev->accel_lsb_div;
    } else {
        data->accelX = (raw_ax * 1000.0f) / dev->accel_lsb_div;
        data->accelY = (raw_ay * 1000.0f) / dev->accel_lsb_div;
        data->accelZ = (raw_az * 1000.0f) / dev->accel_lsb_div;
    }
    
    if (dev->gyro_unit_rads) {
        data->gyroX = (raw_gx * M_PI / 180.0f) / dev->gyro_lsb_div;
        data->gyroY = (raw_gy * M_PI / 180.0f) / dev->gyro_lsb_div;
        data->gyroZ = (raw_gz * M_PI / 180.0f) / dev->gyro_lsb_div;
    } else {
        data->gyroX = (float)raw_gx / dev->gyro_lsb_div;
        data->gyroY = (float)raw_gy / dev->gyro_lsb_div;
        data->gyroZ = (float)raw_gz / dev->gyro_lsb_div;
    }
    
    return qmi8658_read_temp(dev, &data->temperature);
}

esp_err_t qmi8658_is_data_ready(qmi8658_dev_t *dev, bool *ready) {
    if (!dev || !ready) return ESP_ERR_INVALID_ARG;
    
    uint8_t status;
    esp_err_t ret = qmi8658_read_register(dev, QMI8658_STATUS0, &status, 1);
    if (ret != ESP_OK) return ret;
    
    *ready = (status & 0x03) != 0;
    return ESP_OK;
}

esp_err_t qmi8658_get_who_am_i(qmi8658_dev_t *dev, uint8_t *who_am_i) {
    if (!dev || !who_am_i) return ESP_ERR_INVALID_ARG;
    return qmi8658_read_register(dev, QMI8658_WHO_AM_I, who_am_i, 1);
}

esp_err_t qmi8658_reset(qmi8658_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return qmi8658_write_register(dev, QMI8658_CTRL1, 0x80);
}

void qmi8658_set_accel_unit_mps2(qmi8658_dev_t *dev, bool use_mps2) {
    if (dev) {
        dev->accel_unit_mps2 = use_mps2;
    }
}

void qmi8658_set_accel_unit_mg(qmi8658_dev_t *dev, bool use_mg) {
    if (dev) {
        dev->accel_unit_mps2 = !use_mg;
    }
}

void qmi8658_set_gyro_unit_rads(qmi8658_dev_t *dev, bool use_rads) {
    if (dev) {
        dev->gyro_unit_rads = use_rads;
    }
}

void qmi8658_set_gyro_unit_dps(qmi8658_dev_t *dev, bool use_dps) {
    if (dev) {
        dev->gyro_unit_rads = !use_dps;
    }
}

void qmi8658_set_display_precision(qmi8658_dev_t *dev, int decimals) {
    if (dev && decimals >= 0 && decimals <= 10) {
        dev->display_precision = decimals;
    }
}

void qmi8658_set_display_precision_enum(qmi8658_dev_t *dev, qmi8658_precision_t precision) {
    if (dev) {
        dev->display_precision = (int)precision;
    }
}

int qmi8658_get_display_precision(qmi8658_dev_t *dev) {
    return dev ? dev->display_precision : 0;
}

bool qmi8658_is_accel_unit_mps2(qmi8658_dev_t *dev) {
    return dev ? dev->accel_unit_mps2 : false;
}

bool qmi8658_is_accel_unit_mg(qmi8658_dev_t *dev) {
    return dev ? !dev->accel_unit_mps2 : false;
}

bool qmi8658_is_gyro_unit_rads(qmi8658_dev_t *dev) {
    return dev ? dev->gyro_unit_rads : false;
}

bool qmi8658_is_gyro_unit_dps(qmi8658_dev_t *dev) {
    return dev ? !dev->gyro_unit_rads : false;
}

esp_err_t qmi8658_enable_wake_on_motion(qmi8658_dev_t *dev, uint8_t threshold) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = qmi8658_enable_sensors(dev, QMI8658_DISABLE_ALL);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_set_accel_range(dev, QMI8658_ACCEL_RANGE_2G);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_set_accel_odr(dev, QMI8658_ACCEL_ODR_LOWPOWER_21HZ);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_write_register(dev, 0x0B, threshold);
    if (ret != ESP_OK) return ret;
    
    ret = qmi8658_write_register(dev, 0x0C, 0x00);
    if (ret != ESP_OK) return ret;
    
    return qmi8658_enable_sensors(dev, QMI8658_ENABLE_ACCEL);
}

esp_err_t qmi8658_disable_wake_on_motion(qmi8658_dev_t *dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = qmi8658_enable_sensors(dev, QMI8658_DISABLE_ALL);
    if (ret != ESP_OK) return ret;
    
    return qmi8658_write_register(dev, 0x0B, 0x00);
}

esp_err_t qmi8658_write_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t value) {
    if (!dev || !dev->dev_handle) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(dev->dev_handle, data, 2, 1000);
}

esp_err_t qmi8658_read_register(qmi8658_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (!dev || !buffer || length == 0 || !dev->dev_handle) return ESP_ERR_INVALID_ARG;
    
    return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, buffer, length, 1000);
}