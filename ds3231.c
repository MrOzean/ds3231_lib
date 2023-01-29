#include "ds3231.h"

/*!
 * @brief convert decimal to BCD (Binary Coded Decimal)
 */
static inline uint8_t dec_to_BCD(uint8_t value)
{
    return ((value / 10) << 4) | (value % 10);
}
/*!
 * @brief convert BCD(Binary Coded Decimal) to Decimal
 */
static inline uint8_t BCD_to_dec(uint8_t value)
{
    return ((value >> 4) * 10 + (0x0F & value));
}

/*!
 * @brief Read control register value
 *
 * @param[in]  clock         : Pointer to ds3231 device
 * @param[in]  reg_addr      : Register address (DS3231 has two of it)
 *
 * @retval Byte with register value
 */
static uint8_t read_control_reg(ds3231_dev *clock, ds3231_control_reg reg_addr)
{
    uint8_t buff[1];

    clock->read(reg_addr, (uint8_t *)&buff, 1, clock->i2c_chip_ptr);

    return buff[0];
}

/*!
 * @brief Write control register value
 *
 * @param[in]  clock          Pointer to ds3231 device
 * @param[in]  reg_addr       Register address (DS3231 has two of it)
 * @param[in]  value          New value
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 */
static uint8_t write_control_reg(ds3231_dev *clock, ds3231_control_reg reg_addr, uint8_t value)
{
    uint8_t buff[] = {value};

    int8_t res = clock->write(reg_addr, (uint8_t *)&buff, 1, clock->i2c_chip_ptr);

    return buff[0];
}

/*!
 * @brief Set time to target clock
 *
 * @param[in]  clock         : Pointer to ds3231 device
 * @param[out] date_time     : Pointer to ds3231_date_time struct in which data to be written
 *                            is stored.
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
int8_t ds3231_set_date_time(ds3231_dev *clock, ds3231_date_time *date_time)
{
    uint8_t buff[7];

    buff[0] = dec_to_BCD(date_time->seconds);
    buff[1] = dec_to_BCD(date_time->minutes);
    buff[2] = dec_to_BCD(date_time->hours);
    buff[3] = dec_to_BCD(date_time->day_of_week);
    buff[4] = dec_to_BCD(date_time->date);
    buff[5] = dec_to_BCD(date_time->month);
    buff[6] = dec_to_BCD(date_time->year - 2000);

    int8_t res = clock->write(DS3231_CLOCK_REG_ADDR, (uint8_t *)&buff, 7, clock->i2c_chip_ptr);

    return res;
}

/*!
 * @brief Get time from target clock
 *
 * @param[in]  clock         : Pointer to ds3231 device
 * @param[out] date_time     : Pointer to ds3231_date_time struct in which data to be written
 *                            is stored.
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
int8_t ds3231_get_date_time(ds3231_dev *clock, ds3231_date_time *date_time)
{
    uint8_t buff[7];

    int8_t res = clock->read(DS3231_CLOCK_REG_ADDR, (uint8_t *)&buff, 7, clock->i2c_chip_ptr);

    date_time->seconds = BCD_to_dec(buff[0]);
    date_time->minutes = BCD_to_dec(buff[1]);
    date_time->hours = BCD_to_dec(buff[2]);
    date_time->day_of_week = BCD_to_dec(buff[3]);
    date_time->date = BCD_to_dec(buff[4]);
    date_time->month = BCD_to_dec(buff[5]);
    date_time->year = BCD_to_dec(buff[6]) + 2000;

    return res;
}

/*!
 * @brief Get time from target clock
 *
 * @param[in]  clock         : Pointer to ds3231 device
 * @param[out] date_time     : Pointer to ds3231_date_time struct in which data to be written
 *                            is stored.
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
int8_t ds3231_set_alarm_1(ds3231_dev *clock, ds3231_alarm_1 *alarm)
{
    uint8_t buff[4]; // buffer to write

    uint8_t A1M1 = (alarm->alarm_rate & 1) << 7;         // get a1m1 mask
    uint8_t A1M2 = ((alarm->alarm_rate >> 1) & 1) << 7;  // get a1m2 mask
    uint8_t A1M3 = ((alarm->alarm_rate >> 2) & 1) << 7;  // get a1m3 mask
    uint8_t A1M4 = ((alarm->alarm_rate >> 3) & 1) << 7;  // get a1m4 mask
    uint8_t DY_DT = ((alarm->alarm_rate >> 4) & 1) << 6; // get DY/DT but, 1 if only "Alarm when day, hours, minutes, and seconds match"

    // convert input values to BCD and apply mask
    buff[0] = dec_to_BCD(alarm->seconds) | A1M1;
    buff[1] = dec_to_BCD(alarm->minutes) | A1M2;
    buff[2] = dec_to_BCD(alarm->hours) | A1M3; // valid only for 24h format
    buff[3] = dec_to_BCD(alarm->day_date) | A1M4 | DY_DT;

    int8_t res = clock->write(DS3231_ALARM1_REG_ADDRESS, (uint8_t *)&buff, 4, clock->i2c_chip_ptr);

    return res;
}

int8_t ds3231_get_alarm_1(ds3231_dev *clock, ds3231_alarm_1 *alarm)
{
    uint8_t buff[4];

    alarm->alarm_rate = 0;

    int8_t res = clock->read(DS3231_ALARM1_REG_ADDRESS, (uint8_t *)&buff, 4, clock->i2c_chip_ptr);

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

    return res;
}

int8_t ds3231_set_alarm_2(ds3231_dev *clock, ds3231_alarm_2 *alarm)
{
    uint8_t buff[3];

    uint8_t a2m2 = (alarm->alarm_rate & 1) << 7;
    uint8_t a2m3 = ((alarm->alarm_rate >> 1) & 1) << 7;
    uint8_t a2m4 = ((alarm->alarm_rate >> 2) & 1) << 7;
    uint8_t dy_dt = ((alarm->alarm_rate >> 3) & 1) << 6;

    buff[0] = dec_to_BCD(alarm->minutes) | a2m2;
    buff[1] = dec_to_BCD(alarm->hours) | a2m3;
    buff[2] = dec_to_BCD(alarm->day_date) | a2m4 | dy_dt;

    int8_t res = clock->write(DS3231_ALARM2_REG_ADDRESS, (uint8_t *)&buff, sizeof(buff), clock->i2c_chip_ptr);

    return res;
}

int8_t ds3231_get_alarm_2(ds3231_dev *clock, ds3231_alarm_2 *alarm)
{
    uint8_t buff[3];

    alarm->alarm_rate = 0;

    int8_t res = clock->read(DS3231_ALARM2_REG_ADDRESS, (uint8_t *)&buff, 3, clock->i2c_chip_ptr);

    uint8_t a2m2 = (buff[0] >> 7) & 1;
    uint8_t a2m3 = ((buff[1] >> 7) & 1) << 1;
    uint8_t a2m4 = ((buff[2] >> 7) & 1) << 2;
    uint8_t dy_dt = ((buff[2] >> 6) & 1) << 4;

    alarm->alarm_rate = a2m2 | a2m3 | a2m4 | dy_dt;

    alarm->minutes = BCD_to_dec(buff[0] & 0x7F);
    alarm->hours = BCD_to_dec(buff[1] & 0x3F);
    alarm->day_date = BCD_to_dec(buff[2] & 0x3F);

    return res;
}

bool ds3231_check_alarm_interrupt(ds3231_dev *clock, ds3231_alarm alarm)
{
    uint8_t data = read_control_reg(clock, CONTROL_REG_1);
    if (alarm == ALARM_1)
        return (data >> DS3231_A1IE_A1F_BIT_NO) & 1;
    else
        return (data >> DS3231_A2IE_A2F_BIT_NO) & 1;
}

int8_t ds3231_set_alarm_interrupt(ds3231_dev *clock, ds3231_alarm alarm, bool enable)
{
    uint8_t data = read_control_reg(clock, CONTROL_REG_1);
    uint8_t offset = alarm == ALARM_1 ? DS3231_A1IE_A1F_BIT_NO : DS3231_A2IE_A2F_BIT_NO;

    data &= ~(1 << offset);   // clear config
    data |= enable << offset; // set enable

    return write_control_reg(clock, CONTROL_REG_1, data);
}

bool ds3231_check_alarm_fired(ds3231_dev *clock, ds3231_alarm alarm)
{
    uint8_t data = read_control_reg(clock, CONTROL_REG_2);
    if (alarm == ALARM_1)
        return (data >> DS3231_A1IE_A1F_BIT_NO) & 1;
    else
        return (data >> DS3231_A2IE_A2F_BIT_NO) & 1;
}

uint8_t ds3231_reset_alarm_fired(ds3231_dev *clock, ds3231_alarm alarm)
{
    uint8_t data = read_control_reg(clock, CONTROL_REG_2);
    uint8_t offset = alarm == ALARM_1 ? DS3231_A1IE_A1F_BIT_NO : DS3231_A2IE_A2F_BIT_NO;

    data &= ~(1 << offset); // set 0 to alarm fired bit

    return write_control_reg(clock, CONTROL_REG_2, data);
}

int8_t ds3231_set_intcn_mode(ds3231_dev *clock, ds3231_intcn intcn_value)
{
    uint8_t data = read_control_reg(clock, CONTROL_REG_1);
    data &= ~(1 << DS3231_INTCN_BIT_NO);        // clear config
    data |= intcn_value << DS3231_INTCN_BIT_NO; // set enable

    return write_control_reg(clock, CONTROL_REG_1, data);
}

float ds3231_get_temperature(ds3231_dev *clock)
{
    uint8_t buff[2];

    int8_t res = clock->read(DS3231_TEMP_REG_ADDR, (uint8_t *)&buff, 2, clock->i2c_chip_ptr);
    return (float)buff[0] + ((buff[1] >> 6) * 0.25f);
}
