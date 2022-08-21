
#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

/* dependent library header */
#include "bme68x.h"
#include "bsec_interface.h"

#define SAMPLE_COUNT  UINT16_C(300)

int g_i2cFid; // I2C Linux device handle
int i2c_address = BME68X_I2C_ADDR_HIGH;

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    BME68X_INTF_RET_TYPE rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t reg[1];
    reg[0] = reg_addr;

    if (write(g_i2cFid, reg, 1) != 1)
    {
        perror("user_i2c_read_reg");
        rslt = 1;
    }

    if (read(g_i2cFid, reg_data, len) != len)
    {
        perror("user_i2c_read_data");
        rslt = 1;
    }

    return rslt;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    BME68X_INTF_RET_TYPE rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t reg[16];
    reg[0] = reg_addr;
    int i;

    for (i = 1; i < len + 1; i++)
        reg[i] = reg_data[i - 1];

    if (write(g_i2cFid, reg, len + 1) != len + 1)
    {
        perror("user_i2c_write");
        rslt = 1;
        exit(1);
    }

    return rslt;    
}

// open the Linux device
void i2cOpen()
{
    g_i2cFid = open("/dev/i2c-1", O_RDWR);
    if (g_i2cFid < 0)
    {
        perror("i2cOpen");
        exit(1);
    }
}

// set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address)
{
    if (ioctl(g_i2cFid, I2C_SLAVE, address) < 0)
    {
        perror("i2cSetAddress");
        exit(1);
    }
}

// close the Linux device
void i2cClose()
{
    close(g_i2cFid);
}

/*!
 * @brief Delay function
 *
 * @param period - The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    struct timespec ts;
    ts.tv_sec = period / 1000000;
    /* mod because nsec must be in the range 0 to 999999999 */
    ts.tv_nsec = (period % 1000000) * 1000ul;
    nanosleep(&ts, NULL);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int64_t get_current_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (ts.tv_sec * 1e9) + ts.tv_nsec;
}

int main()
{
    i2cOpen();
    i2cSetAddress(i2c_address);

    struct bme68x_dev bme;
    int8_t rslt;
    int8_t status;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint32_t del_period;
    uint8_t n_fields;
    uint16_t sample_count = 1;

    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];

    bme.read = bme68x_i2c_read;
    bme.write = bme68x_i2c_write;
    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = &i2c_address;
    bme.delay_us = bme68x_delay_us;

    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    status = bsec_init();
    if (status != BSEC_OK) {
        printf("bsec_init Error [%d]", status);
        return status;
    }

    rslt = bme68x_get_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    int64_t current_time_ns = get_current_time_ns();

    status = bsec_sensor_control(current_time_ns, &conf);
    if (status != BSEC_OK) {
        printf("bsec_sensor_control Error [%d]", status);
        return status;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    printf("Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");

    while (sample_count <= SAMPLE_COUNT)
    {
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", rslt);

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", rslt);

        if (n_fields)
        {
            printf("%.2f, %.2f, %.2f, %.2f, 0x%x\n",
                data.temperature,
                data.pressure,
                data.humidity,
                data.gas_resistance,
                data.status);
        }

        bme68x_delay_us(2000000, NULL);
        sample_count++;
    }
    // rslt = bme68x_selftest_check(&bme);
    // bme68x_check_rslt("bme68x_selftest_check", rslt);

    // if (rslt == BME68X_OK)
    // {
    //     printf("Self-test passed\n");
    // }

    i2cClose();

    return 0;
}