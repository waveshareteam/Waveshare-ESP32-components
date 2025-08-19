#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "custom_io_expander_ch32v003.h"
#include "pcf85063a.h"

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

static const char *TAG = "pcf85063a_example";

// Initial RTC time to be set
static pcf85063a_datetime_t Set_Time = {
    .year = 2025,
    .month = 07,
    .day = 30,
    .dotw = 3,   // Day of the week: 0 = Sunday
    .hour = 9,
    .min = 0,
    .sec = 0
};

// Alarm time to be set
static pcf85063a_datetime_t Set_Alarm_Time = {
    .year = 2025,
    .month = 07,
    .day = 30,
    .dotw = 3,
    .hour = 9,
    .min = 0,
    .sec = 2
};

char datetime_str[256];  // Buffer to store formatted date-time string


static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true
    };

    return i2c_new_master_bus(&bus_config, bus_handle);
}

static void pcf85063a_test_task(void *arg) {
    i2c_master_bus_handle_t bus_handle = (i2c_master_bus_handle_t)arg;
    pcf85063a_dev_t dev;
    pcf85063a_datetime_t Now_time;
    static esp_io_expander_handle_t custom_io_expander = NULL; // Custom IO expander ch32v003 handle

    custom_io_expander_new_i2c_ch32v003(bus_handle, CUSTOM_IO_EXPANDER_I2C_CH32V003_ADDRESS, &custom_io_expander);
    ESP_LOGI(TAG, "Initializing PCF85063A...");
    esp_err_t ret = pcf85063a_init(&dev, bus_handle, PCF85063A_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCF85063A (error: %d)", ret);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Set current time.");
    pcf85063a_set_time_date(&dev, Set_Time);

    ESP_LOGI(TAG, "Set alarm time.");
    pcf85063a_set_alarm(&dev, Set_Alarm_Time);

    ESP_LOGI(TAG, "Enable alarm interrupt.");
    pcf85063a_enable_alarm(&dev);

    while (1) {
        
        // Read current time from RTC
        pcf85063a_get_time_date(&dev, &Now_time);

        // Format current time as a string
        pcf85063a_datetime_to_str(datetime_str, Now_time);
        ESP_LOGI(TAG, "Now_time is %s", datetime_str);

        // Poll external IO pin for alarm (low level = alarm triggered)
        uint8_t RTC_INT = 1;
        custom_io_expander_get_int(custom_io_expander, &RTC_INT);
        if (RTC_INT == 0)
        {
            // Re-enable alarm if repeated alarms are required
            pcf85063a_enable_alarm(&dev);
            ESP_LOGI(TAG, "The alarm clock goes off.");
        }

        // Wait for 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_master_init(&bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C (error: %d)", ret);
        return;
    }

    xTaskCreate(pcf85063a_test_task, "pcf85063a_test_task", 4096, bus_handle, 5, NULL);
}