#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DS3231_CLOCK_REG_ADDR 0x00 // start address of registers
#define DS3231_ALARM1_REG_ADDRESS 0x07
#define DS3231_ALARM2_REG_ADDRESS 0x0B
#define DS3231_TEMP_REG_ADDR 0x11

#define DS3231_INTCN_BIT_NO 2    // index of INTCN bit right to left
#define DS3231_A2IE_A2F_BIT_NO 1 // index of A2IE bit right to left
#define DS3231_A1IE_A1F_BIT_NO 0 // index of A1IE bit right to left

#define DS3231_I2C_MASTER_TIMEOUT_MS 1000

    /*!
     * @brief Bus communication function pointer which should be mapped to
     * the platform specific read functions of the user
     *
     * @param[in] reg_addr       : Register address from which data is read.
     * @param[out] reg_data     : Pointer to data buffer where read data is stored.
     * @param[in] len            : Number of bytes of data to be read.
     * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
     *                                  for interface related call backs.
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     *
     */
    typedef int8_t (*ds3231_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

    /*!
     * @brief Bus communication function pointer which should be mapped to
     * the platform specific write functions of the user
     *
     * @param[in] reg_addr      : Register address to which the data is written.
     * @param[in] reg_data     : Pointer to data buffer in which data to be written
     *                            is stored.
     * @param[in] len           : Number of bytes of data to be written.
     * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
     *                            for interface related call backs
     *
     * @retval   0   -> Success.
     * @retval Non zero value -> Fail.
     *
     */
    typedef int8_t (*ds3231_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                          void *intf_ptr);

    /*!
     * @brief ds3231 device
     */
    typedef struct
    {
        /* Interface function pointer used to enable the device address for I2C */
        void *i2c_chip_ptr;
        /* user defined read I2C funtion pointer */
        ds3231_read_fptr_t read;
        /* user defined write I2C funtion pointer */
        ds3231_write_fptr_t write;
    } ds3231_dev;

    /*!
     * @brief ds3231 date time store
     */
    typedef struct
    {
        uint8_t seconds;     // 0-59
        uint8_t minutes;     // 0-59
        uint8_t hours;       // 0-23 (24 hour)
        uint8_t day_of_week; // 1-7
        uint8_t date;        // 1-31
        uint8_t month;       // 1-12
        uint16_t year;       // 2000-2099
    } ds3231_date_time;

    /*!
     * @brief ds3231 alarm 1 mode enum
     */
    typedef enum
    {
        A1_ONCE_PER_SECOND = 0x0F,                        // once per second
        A1_SECONDS_MATCH = 0x0E,                          // when seconds match
        A1_MINUTES_SECONDS_MATCH = 0x0C,                  // when minutes and seconds match
        A1_HOURS_MINUTES_SECONDS_MATCH = 0x08,            // when hours, minutes and seconds match
        A1_DATE_HOURS_MINUTES_SECONDS_MATCH = 0x00,       // when date, hours, minutes and seconds match
        A1_DAY_OF_WEEK_HOURS_MINUTES_SECONDS_MATCH = 0x10 // when day of week, hours, minutes and seconds match
    } ds3231_alarm_1_alarm_rate;

    /*!
     * @brief ds3231 alarm 1 configuration
     */
    typedef struct
    {
        uint8_t seconds;
        uint8_t minutes;
        uint8_t hours;
        uint8_t day_date; // Range 1-7 // Range 0-31
        ds3231_alarm_1_alarm_rate alarm_rate;
    } ds3231_alarm_1;

    /*!
     * @brief ds3231 alarm 2 mode enum
     */
    typedef enum
    {
        A2_ONCE_PER_MINUTE = 0x07,                // once per minute
        A2_MINUTES_MATCH = 0x06,                  // when minutes match
        A2_HOURS_MINUTES_MATCH = 0x04,            // when hours and seconds match
        A2_DATE_HOURS_MINUTES_MATCH = 0x00,       // when date, hours and minutes match
        A2_DAY_OF_WEEK_HOURS_MINUTES_MATCH = 0x08 // when day of week, hours and minutes match
    } ds3231_alarm_2_alarm_rate;

    /*!
     * @brief ds3231 alarm selector
     */
    typedef enum
    {
        ALARM_1 = 0x00,
        ALARM_2 = 0x01
    } ds3231_alarm;

    /*!
     * @brief ds3231 INTCN selector
     */
    typedef enum
    {
        INTCN_SQUARE_WAVE = 0x00,
        INTCN_ALARM_INTERRUPT = 0x01
    } ds3231_intcn;

    /*!
     * @brief ds3231 control registers adrresses
     */
    typedef enum
    {
        CONTROL_REG_1 = 0x0E,
        CONTROL_REG_2 = 0x0F
    } ds3231_control_reg;

    /*!
     * @brief ds3231 alarm 2 configuration
     */
    typedef struct
    {
        uint8_t minutes;
        uint8_t hours;
        uint8_t day_date; // Range 1-7 // Range 0-31
        ds3231_alarm_2_alarm_rate alarm_rate;
    } ds3231_alarm_2;

    /*!
     * @brief Setup datetime
     *
     * @param[in] clock       Clock device
     * @param[in] date_time   Filled date_time struct ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_set_date_time(ds3231_dev *clock, ds3231_date_time *date_time);
    /*!
     * @brief Get datetime from chip
     *
     * @param[in] clock        Clock device
     * @param[out] date_time   Date_time ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_get_date_time(ds3231_dev *clock, ds3231_date_time *date_time);
    /*!
     * @brief Setup first alarm
     *
     * @param[in] clock       Clock device
     * @param[in] date_time   Filled ds3231_alarm_1 struct ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_set_alarm_1(ds3231_dev *clock, ds3231_alarm_1 *alarm);
    /*!
     * @brief Get first alarm settings from chip
     *
     * @param[in] clock       Clock device
     * @param[out] alarm      ds3231_alarm_1 date_time ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_get_alarm_1(ds3231_dev *clock, ds3231_alarm_1 *alarm);
    /*!
     * @brief Setup second alarm
     *
     * @param[in] clock       Clock device
     * @param[in] date_time   Filled ds3231_alarm_2 struct ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_set_alarm_2(ds3231_dev *clock, ds3231_alarm_2 *alarm);
    /*!
     * @brief Get second alarm settings from chip
     *
     * @param[in] clock       Clock device
     * @param[out] alarm      ds3231_alarm_2 date_time ptr
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_get_alarm_2(ds3231_dev *clock, ds3231_alarm_2 *alarm);
    /*!
     * @brief Get alarm flag status
     *
     * @param[in] clock       Clock device
     * @param[in] alarm       Alarm selector
     *
     * @retval   Alarm flag status
     */
    bool ds3231_check_alarm_interrupt(ds3231_dev *clock, ds3231_alarm alarm);
    /*!
     * @brief Get alarm fired flag status
     *
     * @param[in] clock       Clock device
     * @param[in] alarm       Alarm selector
     *
     * @retval   Alarm fired flag status
     */
    bool ds3231_check_alarm_fired(ds3231_dev *clock, ds3231_alarm alarm);
    /*!
     * @brief Reset fired flag. Use to restart alarm with existed settings
     *
     * @param[in] clock       Clock device
     * @param[in] alarm       Alarm selector
     *
     * @retval   Alarm fired flag status
     */
    uint8_t ds3231_reset_alarm_fired(ds3231_dev *clock, ds3231_alarm alarm);
    /*!
     * @brief Enable or disable alarm interrupt
     *
     * @param[in] clock       Clock device
     * @param[in] alarm       Alarm selector
     * @param[in] enable      Enable/disable
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_set_alarm_interrupt(ds3231_dev *clock, ds3231_alarm alarm, bool enable);
    /*!
     * @brief Change INTCN flag status
     *
     * @param[in] clock       Clock device
     * @param[in] intcn_value New Value
     *
     * @retval   0 -> Success.
     * @retval Non zero value -> Fail.
     */
    int8_t ds3231_set_intcn_mode(ds3231_dev *clock, ds3231_intcn intcn_value);
    /*!
     * @brief Get value from chip temperature sensor
     *
     * @param[in] clock       Clock device
     *
     * @retval   Temperature in ℃
     */
    float ds3231_get_temperature(ds3231_dev *clock);

#ifdef __cplusplus
}
#endif /* End of CPP guard */