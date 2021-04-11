#include "ds3231.h"

#define CLOCK_REG_ADDR 0x00 // start address of registers
#define ALARM1_REG_ADDRESS 0x07
#define ALARM2_REG_ADDRESS 0x0B
#define TEMP_REG_ADDR 0x11


// convert decimal to BCD (Binary Coded Decimal)
static inline uint8_t dec_to_BCD(uint8_t value)
{
	return ((value / 10) << 4) | (value % 10);
}
// convert BCD(Binary Coded Decimal) to Decimal
static inline uint8_t BCD_to_dec(uint8_t value)
{
	return ((value >> 4) * 10 + (0x0F & value));
}

DS3231 *clock_store;

static inline void write_data(uint8_t address, uint8_t *data, uint16_t data_length)
{
	while (HAL_I2C_Mem_Write(
			   clock_store->i2c_bus,
			   (uint16_t)clock_store->address,
			   address,
			   I2C_MEMADD_SIZE_8BIT,
			   data,
			   data_length,
			   (uint32_t)1000) != HAL_OK)
	{
	}
}

static inline void read_data(uint8_t address, uint8_t *result, uint16_t result_length)
{
	while (HAL_I2C_Mem_Read(
			   clock_store->i2c_bus,
			   (uint16_t)clock_store->address,
			   address,
			   I2C_MEMADD_SIZE_8BIT,
			   result,
			   result_length,
			   (uint32_t)1000) != HAL_OK)
	{
	}
}

DS3231 ds3231_init(uint8_t address, I2C_HandleTypeDef *i2c_bus)
{
	DS3231 clock;

	clock.address = address;
	clock.i2c_bus = i2c_bus;

	return clock;
}

void ds3231_set_date_time(DS3231 *clock, DS3231_DateTime *date_time)
{
	clock_store = clock;

	uint8_t buff[7];

	buff[0] = dec_to_BCD(date_time->seconds);
	buff[1] = dec_to_BCD(date_time->minutes);
	buff[2] = dec_to_BCD(date_time->hours);
	buff[3] = dec_to_BCD(date_time->day_of_week);
	buff[4] = dec_to_BCD(date_time->date);
	buff[5] = dec_to_BCD(date_time->month);
	buff[6] = dec_to_BCD(date_time->year - 2000);

	write_data(CLOCK_REG_ADDR, (uint8_t *)&buff, 7);
}

void ds3231_get_date_time(DS3231 *clock, DS3231_DateTime *date_time)
{
	clock_store = clock;

	uint8_t buff[7];

	read_data(CLOCK_REG_ADDR, (uint8_t *)&buff, 7);

	date_time->seconds = BCD_to_dec(buff[0]);
	date_time->minutes = BCD_to_dec(buff[1]);
	date_time->hours = BCD_to_dec(buff[2]);
	date_time->day_of_week = BCD_to_dec(buff[3]);
	date_time->date = BCD_to_dec(buff[4]);
	date_time->month = BCD_to_dec(buff[5]);
	date_time->year = BCD_to_dec(buff[6]) + 2000;
}

void ds3231_set_alarm_1(DS3231 *clock, DS3231_Alarm_1 *alarm)
{
	clock_store = clock;

	uint8_t buff[4];

	uint8_t a1m1 = (alarm->alarm_rate & 1) << 7;
	uint8_t a1m2 = ((alarm->alarm_rate >> 1) & 1) << 7;
	uint8_t a1m3 = ((alarm->alarm_rate >> 2) & 1) << 7;
	uint8_t a1m4 = ((alarm->alarm_rate >> 3) & 1) << 7;
	uint8_t dy_dt = ((alarm->alarm_rate >> 4) & 1) << 6;

	buff[0] = dec_to_BCD(alarm->seconds) | a1m1;
	buff[1] = dec_to_BCD(alarm->minutes) | a1m2;
	buff[2] = dec_to_BCD(alarm->hours) | a1m3;
	buff[3] = dec_to_BCD(alarm->day_date) | a1m4 | dy_dt;

	write_data(ALARM1_REG_ADDRESS, (uint8_t *)&buff, 4);
	;
}

void ds3231_get_alarm_1(DS3231 *clock, DS3231_Alarm_1 *alarm)
{
	clock_store = clock;

	uint8_t buff[4];

	alarm->alarm_rate = 0;

	read_data(ALARM1_REG_ADDRESS, (uint8_t *)&buff, 4);

	uint8_t a1m1 = (buff[0] >> 7) & 1;
	uint8_t a1m2 = ((buff[1] >> 7) & 1) << 1;
	uint8_t a1m3 = ((buff[2] >> 7) & 1) << 2;
	uint8_t a1m4 = ((buff[3] >> 7) & 1) << 3;
	uint8_t dy_dt = ((buff[3] >> 6) & 1) << 4;

	alarm->alarm_rate = a1m1 | a1m2 | a1m3 | a1m4 | dy_dt;

	alarm->seconds = BCD_to_dec(buff[0] & 0x7F);
	alarm->minutes = BCD_to_dec(buff[1] & 0x7F);
	alarm->hours = BCD_to_dec(buff[2] & 0x3F);
	alarm->day_date = BCD_to_dec(buff[3] & 0x3F);
}

void ds3231_set_alarm_2(DS3231 *clock, DS3231_Alarm_2 *alarm)
{
	clock_store = clock;

	uint8_t buff[3];

	uint8_t a2m2 = (alarm->alarm_rate & 1) << 7;
	uint8_t a2m3 = ((alarm->alarm_rate >> 1) & 1) << 7;
	uint8_t a2m4 = ((alarm->alarm_rate >> 2) & 1) << 7;
	uint8_t dy_dt = ((alarm->alarm_rate >> 3) & 1) << 6;

	buff[0] = dec_to_BCD(alarm->minutes) | a2m2;
	buff[1] = dec_to_BCD(alarm->hours) | a2m3;
	buff[2] = dec_to_BCD(alarm->day_date) | a2m4 | dy_dt;

	write_data(0x0B, (uint8_t *)&buff, sizeof(buff));
}

void ds3231_get_alarm_2(DS3231 *clock, DS3231_Alarm_2 *alarm)
{
	uint8_t buff[3];

	alarm->alarm_rate = 0;

	read_data(ALARM2_REG_ADDRESS, (uint8_t *)&buff, 3);

	uint8_t a2m2 = (buff[0] >> 7) & 1;
	uint8_t a2m3 = ((buff[1] >> 7) & 1) << 1;
	uint8_t a2m4 = ((buff[2] >> 7) & 1) << 2;
	uint8_t dy_dt = ((buff[2] >> 6) & 1) << 4;

	alarm->alarm_rate = a2m2 | a2m3 | a2m4 | dy_dt;

	alarm->minutes = BCD_to_dec(buff[0] & 0x7F);
	alarm->hours = BCD_to_dec(buff[1] & 0x3F);
	alarm->day_date = BCD_to_dec(buff[2] & 0x3F);
}

float ds3231_get_temperature(DS3231 *clock)
{
	uint8_t buff[2];

	read_data(TEMP_REG_ADDR, (uint8_t *)&buff, 2);
	return (float)buff[0] + ((buff[1] >> 6) * 0.25f);
}
