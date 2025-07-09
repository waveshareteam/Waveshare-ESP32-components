# Sensor QMI8658

[![Component Registry](https://components.espressif.com/components/waveshare/qmi8658/badge.svg)](https://components.espressif.com/components/waveshare/qmi8658)

QMI8658 sensor driver,QMI8658 is High-performance 6-axis MEMS inertial measurement Unit (IMU), integrated with 3-axis gyroscope and 3-axis accelerometer, designed for high-precision motion detection and attitude resolution.

| Sensor controller | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| QMI8658            | I2C                     | qmi8658 | [WIKI](https://www.waveshare.net/w/upload/5/5f/QMI8658C.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency waveshare/qmi8658==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use
### Initialization of the I2C bus
```c
static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true
    };
    return i2c_new_master_bus(&bus_config, bus_handle);
}
```
### Sensor initialization and configuration
```c
esp_err_t ret = qmi8658_init(&dev, bus_handle, QMI8658_ADDRESS_HIGH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize QMI8658 (error: %d)", ret);
        vTaskDelete(NULL);
    }

    qmi8658_set_accel_range(&dev, QMI8658_ACCEL_RANGE_8G);
    qmi8658_set_accel_odr(&dev, QMI8658_ACCEL_ODR_1000HZ);

    qmi8658_set_gyro_range(&dev, QMI8658_GYRO_RANGE_512DPS);
    qmi8658_set_gyro_odr(&dev, QMI8658_GYRO_ODR_1000HZ);
```
### Data units and precision Settings
```c
qmi8658_set_accel_unit_mps2(&dev, true);

qmi8658_set_gyro_unit_rads(&dev, true);
    
qmi8658_set_display_precision(&dev, 4);
```
### Reading and printing data
```c
while (1) {
        bool ready;
        ret = qmi8658_is_data_ready(&dev, &ready);
        if (ret == ESP_OK && ready) {
            ret = qmi8658_read_sensor_data(&dev, &data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Accel: X=%.4f m/s², Y=%.4f m/s², Z=%.4f m/s²",
                         data.accelX, data.accelY, data.accelZ);
                ESP_LOGI(TAG, "Gyro:  X=%.4f rad/s, Y=%.4f rad/s, Z=%.4f rad/s",
                         data.gyroX, data.gyroY, data.gyroZ);
                ESP_LOGI(TAG, "Temp:  %.2f °C, Timestamp: %lu",
                         data.temperature, data.timestamp);
                ESP_LOGI(TAG, "----------------------------------------");
            } else {
                ESP_LOGE(TAG, "Failed to read sensor data (error: %d)", ret);
            }
        } else {
            ESP_LOGE(TAG, "Data not ready or error reading status (error: %d)", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
```

### Running results
```shell
I (768317) qmi8658_example: Accel: X=1.8771 m/s², Y=-0.1269 m/s², Z=-9.7807 m/s²
I (768317) qmi8658_example: Gyro:  X=-0.0404 rad/s, Y=0.1039 rad/s, Z=0.0068 rad/s
I (768317) qmi8658_example: Temp:  32.08 °C, Timestamp: 694517
I (768217) qmi8658_example: ----------------------------------------
```