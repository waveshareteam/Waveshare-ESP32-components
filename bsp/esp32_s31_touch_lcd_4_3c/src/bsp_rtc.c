/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/esp32_s31_touch_lcd_4_3c.h"

static pcf85063a_dev_t rtc_dev;

esp_err_t bsp_rtc_init(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    return pcf85063a_init(&rtc_dev, bsp_i2c_get_handle(), BSP_RTC_I2C_ADDRESS);
}

esp_err_t bsp_set_rtc_time_date(pcf85063a_datetime_t time)
{
    return pcf85063a_set_time_date(&rtc_dev, time);
}

esp_err_t bsp_set_rtc_alarm_time(pcf85063a_datetime_t time)
{
    return pcf85063a_set_alarm(&rtc_dev, time);
}

esp_err_t bsp_get_rtc_time_date(pcf85063a_datetime_t *time)
{
    return pcf85063a_get_time_date(&rtc_dev, time);
}

esp_err_t bsp_get_rtc_alarm_time(pcf85063a_datetime_t *time)
{
    return pcf85063a_get_alarm(&rtc_dev, time);
}

esp_err_t bsp_enable_rtc_alarm(void)
{
    return pcf85063a_enable_alarm(&rtc_dev);
}

esp_err_t bsp_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time)
{
    pcf85063a_datetime_to_str(datetime_str, 32, time);
    return ESP_OK;
}
