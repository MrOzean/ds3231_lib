#include "stm32f1xx.h"

#ifndef DS3231_LIB_INC_DS3231_H_
#define DS3231_LIB_INC_DS3231_H_

typedef struct {
	// chip address
	uint8_t address;
	// pointer to i2c handle
	I2C_HandleTypeDef *i2c_bus;
} DS3231;

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;		  // 0-23 (24 hour); 1-12 (12 hour)
	uint8_t day_of_week;  // 1-7
	uint8_t date; 		  // 1-31
	uint8_t month;
	uint8_t year;         //  0-99
} DS3231_DateTime;

typedef enum {
	A1_ONCE_PER_SECOND = 0x0F, // once per second
	A1_SECONDS_MATCH = 0x0E, // when seconds match
	A1_MINUTES_SECONDS_MATCH = 0x0C, // when minutes and seconds match
	A1_HOURS_MINUTES_SECONDS_MATCH = 0x08, // when hours, minutes and seconds match
	A1_DATE_HOURS_MINUTES_SECONDS_MATCH = 0x00, // when date, hours, minutes and seconds match
	A1_DAY_OF_WEEK_HOURS_MINUTES_SECONDS_MATCH = 0x10 // when day of week, hours, minutes and seconds match
} DS3231_Alarm_1_Alarm_Rate;

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_date;  // Range 1-7 // Range 0-31
	DS3231_Alarm_1_Alarm_Rate alarm_rate;
} DS3231_Alarm_1;

typedef enum {
	A2_ONCE_PER_MINUTE = 0x07, // once per minute
	A2_MINUTES_MATCH = 0x06, // when minutes match
	A2_HOURS_MINUTES_MATCH = 0x04, // when hours and seconds match
	A2_DATE_HOURS_MINUTES_MATCH = 0x00, // when date, hours and minutes match
	A2_DAY_OF_WEEK_HOURS_MINUTES_MATCH = 0x08 // when day of week, hours and minutes match
} DS3231_Alarm_2_Alarm_Rate;

typedef struct {
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_date;   // Range 1-7 // Range 0-31
	DS3231_Alarm_2_Alarm_Rate alarm_rate;
} DS3231_Alarm_2;

DS3231 DS3231_init(uint8_t address, I2C_HandleTypeDef *i2c_bus);

void DS3231_set_date_time(DS3231* clock, DS3231_DateTime* date_time);
void DS3231_get_date_time(DS3231* clock, DS3231_DateTime* date_time);

void DS3231_set_alarm_1(DS3231* clock, DS3231_Alarm_1* alarm);
void DS3231_get_alarm_1(DS3231* clock, DS3231_Alarm_1* alarm);

void DS3231_enable_alarm_1(DS3231* clock);
void DS3231_disable_alarm_1(DS3231* clock);

void DS3231_set_alarm_2(DS3231* clock, DS3231_Alarm_2* alarm);
void DS3231_get_alarm_2(DS3231* clock, DS3231_Alarm_2* alarm);

void DS3231_enable_alarm_2(DS3231* clock);
void DS3231_disable_alarm_2(DS3231* clock);

float DS3231_get_temperature(DS3231* clock);

#endif /* DS3231_LIB_INC_DS3231_H_ */
