#ifndef PCF85063A_H
#define PCF85063A_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCF85063A_LIBRARY_VERSION "1.0.0"

#define PCF85063A_ADDRESS  0x51

/* 
 * Year offset 
 */
#define YEAR_OFFSET			(1970)

/* 
 * Register overview - control & status registers 
 */
#define PCF85063A_RTC_CTRL_1_ADDR     (0x00) /*!< Control Register 1 Address  */
#define PCF85063A_RTC_CTRL_2_ADDR     (0x01) /*!< Control Register 2 Address  */
#define PCF85063A_RTC_OFFSET_ADDR     (0x02) /*!< Offset Register Address  */
#define PCF85063A_RTC_RAM_by_ADDR     (0x03) /*!< RAM Register Address  */

/* 
 * Register overview - time & data registers 
 */
#define PCF85063A_RTC_SECOND_ADDR		(0x04) /*!< Seconds Register Address  */
#define PCF85063A_RTC_MINUTE_ADDR		(0x05) /*!< Minutes Register Address  */
#define PCF85063A_RTC_HOUR_ADDR		(0x06) /*!< Hours Register Address  */
#define PCF85063A_RTC_DAY_ADDR		(0x07) /*!< Day Register Address  */
#define PCF85063A_RTC_WDAY_ADDR		(0x08) /*!< Weekday Register Address  */
#define PCF85063A_RTC_MONTH_ADDR		(0x09) /*!< Month Register Address  */
#define PCF85063A_RTC_YEAR_ADDR		(0x0A)	/*!< Year Register Address (0-99); real year = 1970 + RCC register year */

/* 
 * Register overview - alarm registers 
 */
#define PCF85063A_RTC_SECOND_ALARM	(0x0B) /*!< Alarm Seconds Register Address  */
#define PCF85063A_RTC_MINUTE_ALARM	(0x0C) /*!< Alarm Minutes Register Address  */
#define PCF85063A_RTC_HOUR_ALARM		(0x0D) /*!< Alarm Hours Register Address  */
#define PCF85063A_RTC_DAY_ALARM		(0x0E) /*!< Alarm Day Register Address  */
#define PCF85063A_RTC_WDAY_ALARM		(0x0F) /*!< Alarm Weekday Register Address  */

/* 
 * Register overview - timer registers 
 */
#define PCF85063A_RTC_TIMER_VAL 	    (0x10) /*!< Timer Value Register Address  */
#define PCF85063A_RTC_TIMER_MODE	    (0x11) /*!< Timer Mode Register Address  */

/* 
 * PCF85063A_RTC_CTRL_1 Register 
 */
#define PCF85063A_RTC_CTRL_1_EXT_TEST (0x80) /*!< External Test  */
#define PCF85063A_RTC_CTRL_1_STOP     (0x20) // 0-RTC clock runs, 1-RTC clock is stopped 
#define PCF85063A_RTC_CTRL_1_SR       (0X10) // 0-No software reset, 1-Initiate software reset 
#define PCF85063A_RTC_CTRL_1_CIE      (0X04) // 0-No correction interrupt generated, 1-Interrupt pulses are generated at every correction cycle 
#define PCF85063A_RTC_CTRL_1_CAP_SEL  (0X01) // 0-7PF, 1-12.5PF / 0-7PF，1-12.5PF

/* 
 * PCF85063A_RTC_CTRL_2 Register 
 */
#define PCF85063A_RTC_CTRL_2_AIE      (0X80) // Alarm Interrupt 0-Disable, 1-Enable 
#define PCF85063A_RTC_CTRL_2_AF       (0X40) // Alarm Flag 0-Inactive/Cleared, 1-Active/Unchanged 
#define PCF85063A_RTC_CTRL_2_MI       (0X20) // Minute Interrupt 0-Disable, 1-Enable 
#define PCF85063A_RTC_CTRL_2_HMI      (0X10) // Half Minute Interrupt 
#define PCF85063A_RTC_CTRL_2_TF       (0X08) // Timer Flag 

#define PCF85063A_RTC_OFFSET_MODE     (0X80) // Offset Mode 

#define PCF85063A_RTC_TIMER_MODE_TE   (0X04) // Timer Enable 0-Disable, 1-Enable 
#define PCF85063A_RTC_TIMER_MODE_TIE  (0X02) // Timer Interrupt Enable 0-Disable, 1-Enable 
#define PCF85063A_RTC_TIMER_MODE_TI_TP (0X01) // Timer Interrupt Mode 0-Interrupt follows Timer Flag, 1-Interrupt generates a pulse 


// Format 
#define PCF85063A_RTC_ALARM 			(0x80)	// Set AEN_x registers 
#define PCF85063A_RTC_CTRL_1_DEFAULT	(0x00) // PCF85063A_RTC_CTRL_1 Default Value 
#define PCF85063A_RTC_CTRL_2_DEFAULT	(0x00) // PCF85063A_RTC_CTRL_2 Default Value 

#define PCF85063A_RTC_TIMER_FLAG		(0x08) // Timer Flag 


/* 
 * DateTime structure 
 */
typedef struct {
    uint16_t year;     // Year 
    uint8_t month;    // Month 
    uint8_t day;      // Day 
    uint8_t dotw;     // Day of the Week 
    uint8_t hour;     // Hour 
    uint8_t min;      // Minute 
    uint8_t sec;      // Second 
} pcf85063a_datetime_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

} pcf85063a_dev_t;

/**
 * Initialize PCF85063A 
 **/
esp_err_t pcf85063a_init(pcf85063a_dev_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

/**
 * Software reset PCF85063A 
 **/
esp_err_t pcf85063a_reset(pcf85063a_dev_t *dev);

/**
 * Set RTC time 
 **/
esp_err_t pcf85063a_set_time(pcf85063a_dev_t *dev, pcf85063a_datetime_t time);

/**
 * Set RTC date 
 **/
esp_err_t pcf85063a_set_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t date);

/**
 * Set both RTC time and date 
 **/
esp_err_t pcf85063a_set_time_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t time);

/**
 * Read current RTC time and date 
 **/
esp_err_t pcf85063a_get_time_date(pcf85063a_dev_t *dev, pcf85063a_datetime_t *time);

/**
 * Enable Alarm and Clear Alarm flag 
 **/
esp_err_t pcf85063a_enable_alarm(pcf85063a_dev_t *dev);

/**
 * Get Alarm flag 
 **/
esp_err_t pcf85063a_get_alarm_flag(pcf85063a_dev_t *dev, uint8_t *Value);

/**
 * Set Alarm time 
 **/
esp_err_t pcf85063a_set_alarm(pcf85063a_dev_t *dev, pcf85063a_datetime_t time);

/**
 * Read the alarm time set 
 **/
esp_err_t pcf85063a_get_alarm(pcf85063a_dev_t *dev, pcf85063a_datetime_t *time);

/**
 * Convert time to string 
 **/
void pcf85063a_datetime_to_str(char *datetime_str, pcf85063a_datetime_t time);

esp_err_t  pcf85063a_write_register(pcf85063a_dev_t *dev, uint8_t *pdata, uint8_t length);

esp_err_t pcf85063a_read_register(pcf85063a_dev_t *dev, uint8_t reg, uint8_t *buffer, uint8_t length);

#endif // PCF85063A_H

#ifdef __cplusplus
}
#endif