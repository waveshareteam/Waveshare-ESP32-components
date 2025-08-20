#include "pcf85063a.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "PCF85063A";

static uint8_t decToBcd(int val);
static int bcdToDec(uint8_t val);

esp_err_t pcf85063a_init(pcf85063a_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr) {
    if (!dev || !bus_handle) return ESP_ERR_INVALID_ARG;
    
    dev->bus_handle = bus_handle;

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
    
    uint8_t buf[2] = {PCF85063A_RTC_CTRL_1_ADDR, PCF85063A_RTC_CTRL_1_DEFAULT | PCF85063A_RTC_CTRL_1_CAP_SEL};
    ret = pcf85063a_write_register(dev, buf, 2);

    ESP_LOGI(TAG, "PCF85063A initialized successfully");
    return ret;
}

esp_err_t pcf85063a_reset(pcf85063a_dev_t *dev) {
    if (!dev ) return ESP_ERR_INVALID_ARG;
    
    uint8_t buf[2] = {PCF85063A_RTC_CTRL_1_ADDR, 
                        PCF85063A_RTC_CTRL_1_DEFAULT | \
                        PCF85063A_RTC_CTRL_1_CAP_SEL | \
                        PCF85063A_RTC_CTRL_1_SR};
    return pcf85063a_write_register(dev, buf, 2);
}

esp_err_t pcf85063a_set_time(pcf85063a_dev_t *dev, pcf85063a_datetime_t time) {
    if (!dev ) return ESP_ERR_INVALID_ARG;
    
    uint8_t buf[4] = {PCF85063A_RTC_SECOND_ADDR,
					  decToBcd(time.sec),
					  decToBcd(time.min),
					  decToBcd(time.hour)};
    return pcf85063a_write_register(dev, buf, 4);
}

esp_err_t pcf85063a_set_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t date)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
	uint8_t buf[5] = {PCF85063A_RTC_DAY_ADDR,
					  decToBcd(date.day),
					  decToBcd(date.dotw),
					  decToBcd(date.month),
					  decToBcd(date.year - YEAR_OFFSET)};
	return pcf85063a_write_register(dev, buf, 5);
}

esp_err_t pcf85063a_set_time_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t time)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
	uint8_t buf[8] = {PCF85063A_RTC_SECOND_ADDR,
					  decToBcd(time.sec),
					  decToBcd(time.min),
					  decToBcd(time.hour),
					  decToBcd(time.day),
					  decToBcd(time.dotw),
					  decToBcd(time.month),
					  decToBcd(time.year - YEAR_OFFSET)};
	return pcf85063a_write_register(dev, buf, 8);
}

esp_err_t pcf85063a_get_time_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t *time)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;
    uint8_t bufss[7] = {0};

    ret = pcf85063a_read_register(dev, PCF85063A_RTC_SECOND_ADDR, bufss, 7);
	time->sec = bcdToDec(bufss[0] & 0x7F);
	time->min = bcdToDec(bufss[1] & 0x7F);
	time->hour = bcdToDec(bufss[2] & 0x3F);
	time->day = bcdToDec(bufss[3] & 0x3F);
	time->dotw = bcdToDec(bufss[4] & 0x07);
	time->month = bcdToDec(bufss[5] & 0x1F);
	time->year = bcdToDec(bufss[6]) + YEAR_OFFSET;

	return ret;
}

esp_err_t pcf85063a_enable_alarm(pcf85063a_dev_t *dev)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
    
    uint8_t buf[2] = {PCF85063A_RTC_CTRL_2_ADDR, 
                        PCF85063A_RTC_CTRL_2_DEFAULT | \
                        PCF85063A_RTC_CTRL_2_AIE};
    buf[1] &= ~PCF85063A_RTC_CTRL_2_AF;                    
    return pcf85063a_write_register(dev, buf, 2);
}

esp_err_t pcf85063a_get_alarm_flag(pcf85063a_dev_t *dev, uint8_t *Value)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;

    ret = pcf85063a_read_register(dev, PCF85063A_RTC_CTRL_2_ADDR, Value, 1);
    *Value &= PCF85063A_RTC_CTRL_2_AF | PCF85063A_RTC_CTRL_2_AIE;

	return ret;
}

esp_err_t pcf85063a_set_alarm(pcf85063a_dev_t *dev, pcf85063a_datetime_t time)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
    
    uint8_t buf[6] = {PCF85063A_RTC_SECOND_ALARM,
		decToBcd(time.sec) & (~PCF85063A_RTC_ALARM),
		decToBcd(time.min) & (~PCF85063A_RTC_ALARM),
		decToBcd(time.hour) & (~PCF85063A_RTC_ALARM),
		PCF85063A_RTC_ALARM,      // Disable day 
		PCF85063A_RTC_ALARM};     // Disable weekday          
    return pcf85063a_write_register(dev, buf, 6);
}

esp_err_t pcf85063a_get_alarm(pcf85063a_dev_t *dev, pcf85063a_datetime_t *time)
{
    if (!dev ) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;
    uint8_t bufss[7] = {0};

    ret = pcf85063a_read_register(dev, PCF85063A_RTC_CTRL_2_ADDR, bufss, 7);
    // Convert the BCD format seconds, minutes, hours, day, and weekday into decimal and store them in the time structure
	time->sec = bcdToDec(bufss[0] & 0x7F);	// Seconds, up to 7 valid bits, mask processing 										
	time->min = bcdToDec(bufss[1] & 0x7F);	// Minutes, up to 7 valid bits, mask processing 										 
	time->hour = bcdToDec(bufss[2] & 0x3F); // Hours, 24-hour format, up to 6 valid bits, mask processing 										 
	time->day = bcdToDec(bufss[3] & 0x3F);	// Date, up to 6 valid bits, mask processing 										
	time->dotw = bcdToDec(bufss[4] & 0x07); // Day of the week, up to 3 valid bits, mask processing 									

	return ret;
}

/**
 * Convert normal decimal numbers to binary coded decimal 
 **/
static uint8_t decToBcd(int val)
{
	return (uint8_t)((val / 10 * 16) + (val % 10));
}

/**
 * Convert binary coded decimal to normal decimal numbers  
 **/
static int bcdToDec(uint8_t val)
{
	return (int)((val / 16 * 10) + (val % 16));
}

/**
 * Convert time to string 
 **/
void pcf85063a_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time)
{
	sprintf(datetime_str, " %d.%d.%d  %d %d:%d:%d ", time.year, time.month,
			time.day, time.dotw, time.hour, time.min, time.sec);
}

esp_err_t pcf85063a_write_register(pcf85063a_dev_t *dev, uint8_t *pdata, uint8_t length) {
    if (!dev || !dev->dev_handle) return ESP_ERR_INVALID_ARG;
    
    return i2c_master_transmit(dev->dev_handle, pdata, length, 1000);
}

esp_err_t pcf85063a_read_register(pcf85063a_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (!dev || !buffer || length == 0 || !dev->dev_handle) return ESP_ERR_INVALID_ARG;
    
    return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, buffer, length, 1000);
}