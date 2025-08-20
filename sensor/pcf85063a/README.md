# Sensor PCF85063A

[![Component Registry](https://components.espressif.com/components/waveshare/pcf85063a/badge.svg)](https://components.espressif.com/components/waveshare/pcf85063a)

PCF85063A sensor driver,PCF85063A is RTC.

| Sensor controller | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| PCF85063A            | I2C                     | qmi8658 | [WIKI](https://files.waveshare.com/wiki/common/PCF85063A.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency waveshare/pcf85063a==1.0.0
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
    esp_err_t ret = qmi8658_init(&dev, bus_handle, PCF85063A_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCF85063A (error: %d)", ret);
        vTaskDelete(NULL);
    }

```
### Set the time and date, as well as the alarm time, and enable the alarm.
```c
    ESP_LOGI(TAG, "Set current time.");
    pcf85063a_set_time_date(&dev, Set_Time);

    ESP_LOGI(TAG, "Set alarm time.");
    pcf85063a_set_alarm(&dev, Set_Alarm_Time);

    ESP_LOGI(TAG, "Enable alarm interrupt.");
    pcf85063a_enable_alarm(&dev);
```
### Read the current time and check if the alarm has been triggered
```c
    while (1) {
        
        // Read current time from RTC
        pcf85063a_get_time_date(&dev, &Now_time);

        // Format current time as a string
        pcf85063a_datetime_to_str(datetime_str, Now_time);
        ESP_LOGI(TAG, "Now_time is %s", datetime_str);

        // Poll external IO pin for alarm (low level = alarm triggered)
        if (gpio_get_level(RTC_INT_PIN) == 0)
        {
            // Re-enable alarm if repeated alarms are required
            pcf85063a_enable_alarm(&dev);
            ESP_LOGI(TAG, "The alarm clock goes off.");
        }

        // Wait for 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
```

### Running results
```shell
I (278) pcf85063a_example: Initializing PCF85063A...
I (288) PCF85063A: PCF85063A initialized successfully
I (288) pcf85063a_example: Set current time.
I (298) pcf85063a_example: Set alarm time.
I (298) pcf85063a_example: Enable alarm interrupt.
I (308) pcf85063a_example: Now_time is  2025.7.30  3 9:0:0 
I (1308) pcf85063a_example: Now_time is  2025.7.30  3 9:0:1 
I (2308) pcf85063a_example: Now_time is  2025.7.30  3 9:0:2 
I (2308) pcf85063a_example: The alarm clock goes off.
I (3308) pcf85063a_example: Now_time is  2025.7.30  3 9:0:3
------------
```