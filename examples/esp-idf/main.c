// alarm 

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "ds3231.h"

#define TAG "DS3231  test"

#define I2C_MASTER_SCL_IO 10        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 9         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define DS3231_CLOCK_ADDR 0x68 /*!< Slave address of the DS3231 clock */

#define ESP_INTR_FLAG_DEFAULT 0
#define ALARM_INPUT_PIN 3

// clock + alarm interrupt
TaskHandle_t ISR = NULL;
ds3231_dev clock;
ds3231_date_time clock_dt;
ds3231_alarm_1 alarm;
uint8_t ds3231_address = DS3231_CLOCK_ADDR;

/**
 * @brief i2c master initialization, platform specific
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
// setup ALARM_INPUT_PIN to hardware interrupt 
void init_gpio(void)
{
    //! set alarm interrupt
    gpio_pad_select_gpio(ALARM_INPUT_PIN);
    // set the correct direction
    gpio_set_direction(ALARM_INPUT_PIN, GPIO_MODE_INPUT);
    // enable interrupt on falling (1->0) edge for button pin
    gpio_set_intr_type(ALARM_INPUT_PIN, GPIO_INTR_NEGEDGE);
    // Install the driver’s GPIO ISR handler service, which allows per-pin GPIO interrupt handlers.
    //  install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // attach the interrupt service routine
    gpio_isr_handler_add(ALARM_INPUT_PIN, alarm_isr_handler, NULL);
    // Create and start stats task
    xTaskCreate(alarm_intrr_task, "alarm_intrr_task", 4096, NULL, 10, &ISR);
}

// user defined i2c read using platform specific functions
int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    int8_t i2c_addr = *((int *)intf_ptr);
    int ret = i2c_master_write_read_device(I2C_MASTER_NUM, i2c_addr, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS);
    return ret;
}
// user defined i2c write using platform specific functions
int8_t i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    int8_t i2c_addr = *((int *)intf_ptr);
    uint8_t write_buf[length + 1];

    write_buf[0] = reg_addr;
    for (size_t i = 0; i < length; i++)
    {
        write_buf[i + 1] = reg_data[i];
    }

    int ret = i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    return ret;
}

// interrupt service routine
void IRAM_ATTR alarm_isr_handler(void *arg)
{
    // alarm fired
    xTaskResumeFromISR(ISR);
}

// task that will react to interrupt
void alarm_intrr_task(void *arg)
{
    while (1)
    {
        vTaskSuspend(NULL);
        ds3231_get_date_time(&clock, &clock_dt); // <- get data
        ESP_LOGI("get data", "%d-%02d-%02d %02d:%02d:%02d, day of week: %d", clock_dt.year, clock_dt.month, clock_dt.date, clock_dt.hours, clock_dt.minutes, clock_dt.seconds, clock_dt.day_of_week);
        ds3231_reset_alarm_fired(&clock, ALARM_1);
    }
}

void init_clock()
{
    //! CLOCK SETUP
    clock.i2c_chip_ptr = &ds3231_address;
    clock.read = i2c_reg_read;
    clock.write = i2c_reg_write;

    //! CLOCK ALARM SETUP
    alarm.alarm_rate = A1_SECONDS_MATCH;
    // alarm.alarm_rate = A1_ONCE_PER_SECOND;
    // alarm.day_date = 0;
    // alarm.hours = 15;
    // alarm.minutes = 15;
    alarm.seconds = 0; // setup alarm 1 to fire every minute at :00 seconds

    ds3231_set_alarm_interrupt(&clock, ALARM_1, false);   // disable existed alarm
    ds3231_set_alarm_1(&clock, &alarm);                   // set new config
    ds3231_set_intcn_mode(&clock, INTCN_ALARM_INTERRUPT); // set INTCN to ALARM_INTERRUPT mode
    ds3231_reset_alarm_fired(&clock, ALARM_1);            // reset alarm if fired anytime
    ds3231_set_alarm_interrupt(&clock, ALARM_1, true);    // enable new alarm

    ESP_LOGI(TAG, "ds3231 initialized successfully");
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    init_clock();
    init_gpio();

    clock_dt.year = 2023;
    clock_dt.month = 1;
    clock_dt.date = 9;
    clock_dt.hours = 16;
    clock_dt.minutes = 21;
    clock_dt.seconds = 50;

    ds3231_set_date_time(&clock, &clock_dt); // setup time

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}