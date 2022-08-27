
#define _POSIX_C_SOURCE 199309L
#define BSEC_CHECK_INPUT(x, shift) (x & (1 << (shift - 1)))

#define NUM_USED_OUTPUTS 10

#define ADDRESS     "tcp://localhost:1883"
#define CLIENTID    "bsec_bme680"
#define TOPIC       "bsec_bme680"
#define QOS         1
#define TIMEOUT     10000L

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
#include "MQTTClient.h"

/* dependent library header */
#include "bme68x.h"
#include "bsec_interface.h"

#define SAMPLE_COUNT UINT16_C(300)

int g_i2cFid; // I2C Linux device handle
int i2c_address = BME68X_I2C_ADDR_HIGH;

float ext_temp_offset = 0.0f;
u_int8_t operating_mode = BME68X_FORCED_MODE;

/*!
 * I2C read function map to COINES platform
 */
uint8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    int dev_addr = *(int *)intf_ptr;

    uint8_t reg[1];
    reg[0] = reg_addr;

    if (write(dev_addr, reg, 1) != 1)
    {
        perror("user_i2c_read_reg");
        rslt = 1;
    }

    if (read(dev_addr, reg_data, len) != len)
    {
        perror("user_i2c_read_data");
        rslt = 1;
    }

    return rslt;
}

/*!
 * I2C write function map to COINES platform
 */
uint8_t bme68x_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    int dev_addr = *(int *)intf_ptr;

    uint8_t reg[16];
    reg[0] = reg_addr;
    int i;

    for (i = 1; i < len + 1; i++)
        reg[i] = reg_data[i - 1];

    if (write(dev_addr, reg, len + 1) != len + 1)
    {
        perror("user_i2c_write");
        rslt = 1;
        exit(1);
    }

    return rslt;
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
        fprintf(stderr, "API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
        break;
    case BME68X_E_COM_FAIL:
        fprintf(stderr, "API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
        break;
    case BME68X_E_INVALID_LENGTH:
        fprintf(stderr, "API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
        break;
    case BME68X_E_DEV_NOT_FOUND:
        fprintf(stderr, "API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
        break;
    case BME68X_E_SELF_TEST:
        fprintf(stderr, "API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
        break;
    case BME68X_W_NO_NEW_DATA:
        fprintf(stderr, "API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
        break;
    default:
        fprintf(stderr, "API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
        break;
    }
}

/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *  
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme680_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;
    
    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    
    bsec_library_return_t status = BSEC_OK;
    
    /* note: Virtual sensors as desired to be added here */
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[7].sample_rate = sample_rate;
    
    requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[8].sample_rate = sample_rate;
    requested_virtual_sensors[9].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[9].sample_rate = sample_rate;
    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);
    
    return status;
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

int64_t get_current_time_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (ts.tv_sec * 1e9) + ts.tv_nsec;
}

/*
 * @brief Sensor field data structure
 */
struct iot_processed_data
{
    /*! Temperature in degree celsius */
    float temperature;
    float raw_temperature;

    /*! Pressure in Pascal */
    float raw_pressure;

    /*! Humidity in % relative humidity x1000 */
    float humidity;
    float raw_humidity;

    float raw_gas;

    float iaq;
    uint8_t iaq_accuracy;

    float static_iaq;
    uint8_t static_iaq_accuracy;

    float co2_equivalent;
    uint8_t co2_accuracy;

    float breath_voc_equivalent;
    uint8_t breath_voc_accuracy;

    float comp_gas_value;
    uint8_t comp_gas_accuracy;

    float gas_percentage;
    uint8_t gas_percentage_acccuracy;

    int64_t timestamp;
};

int8_t iot_bsec_process_data(int64_t current_time_ns, bsec_bme_settings_t *sensor_settings, struct bme68x_data *data, struct iot_processed_data *processed_data)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
    uint8_t num_bsec_outputs = 0;
    uint8_t num_bsec_inputs = 0;
    uint8_t index = 0;
    int64_t timestamp = 0;

    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    memset(&bsec_outputs, 0, sizeof(bsec_outputs));

    // printf("bsec_process_data %d\n", sensor_settings->process_data);

    uint8_t status;

    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[num_bsec_inputs].signal = ext_temp_offset;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }

    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_TEMPERATURE))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[num_bsec_inputs].signal = data->temperature;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }

    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_HUMIDITY))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[num_bsec_inputs].signal = data->humidity;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }

    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[num_bsec_inputs].signal = data->pressure;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }
    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_GASRESISTOR) &&
        (data->status & BME68X_GASM_VALID_MSK))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[num_bsec_inputs].signal = data->gas_resistance;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }
    if (BSEC_CHECK_INPUT(sensor_settings->process_data, BSEC_INPUT_PROFILE_PART) &&
        (data->status & BME68X_GASM_VALID_MSK))
    {
        inputs[num_bsec_inputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[num_bsec_inputs].signal = (operating_mode == BME68X_FORCED_MODE) ? 0 : data->gas_index;
        inputs[num_bsec_inputs].time_stamp = current_time_ns;
        num_bsec_inputs++;
    }

    if (num_bsec_inputs > 0)
    {

        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;

        // printf("bsec_do_steps\n");

        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
        status = bsec_do_steps(inputs, num_bsec_inputs, &bsec_outputs, &num_bsec_outputs);

        /* Iterate through the outputs and extract the relevant ones. */
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                case BSEC_OUTPUT_IAQ:
                    processed_data->iaq = bsec_outputs[index].signal;
                    processed_data->iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_STATIC_IAQ:
                    processed_data->static_iaq = bsec_outputs[index].signal;
                    processed_data->static_iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    processed_data->co2_equivalent = bsec_outputs[index].signal;
                    processed_data->co2_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    processed_data->breath_voc_equivalent = bsec_outputs[index].signal;
                    processed_data->breath_voc_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    processed_data->temperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    processed_data->raw_pressure = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    processed_data->humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    processed_data->raw_gas = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_TEMPERATURE:
                    processed_data->raw_temperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    processed_data->raw_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_GAS_PERCENTAGE:
                    processed_data->gas_percentage = bsec_outputs[index].signal;
                    processed_data->gas_percentage_acccuracy = bsec_outputs[index].accuracy;
                    break;
                default:
                    continue;
            }
            
            /* Assume that all the returned timestamps are the same */
            processed_data->timestamp = bsec_outputs[index].time_stamp;
        }
    }

    return status;
}

int8_t iot_init_bme68x(struct bme68x_dev *bme, struct bme68x_conf *conf, struct bme68x_heatr_conf *heatr_conf) {
    int8_t rslt;

    bme->read = bme68x_i2c_read;
    bme->write = bme68x_i2c_write;
    bme->intf = BME68X_I2C_INTF;
    bme->intf_ptr = &g_i2cFid;
    bme->delay_us = bme68x_delay_us;

    rslt = bme68x_init(bme);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_init", rslt);
        return rslt;
    }

    rslt = bme68x_get_conf(conf, bme);
    bme68x_check_rslt("bme68x_get_conf", rslt);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_get_conf", rslt);
        return rslt;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf->filter = BME68X_FILTER_OFF;
    conf->odr = BME68X_ODR_NONE;
    conf->os_hum = BME68X_OS_16X;
    conf->os_pres = BME68X_OS_1X;
    conf->os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(conf, bme);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_set_conf", rslt);
        return rslt;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf->enable = BME68X_ENABLE;
    heatr_conf->heatr_temp = 300;
    heatr_conf->heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, heatr_conf, bme);    
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
        return rslt;
    }

    return rslt;
}

int8_t iot_update_bme68x_config(struct bme68x_dev *bme, struct bme68x_conf *conf, struct bme68x_heatr_conf *heatr_conf, bsec_bme_settings_t *sensor_settings) {
    int8_t rslt;

    conf->os_hum = sensor_settings->humidity_oversampling;
    conf->os_pres = sensor_settings->pressure_oversampling;
    conf->os_temp = sensor_settings->temperature_oversampling;
    rslt = bme68x_set_conf(conf, bme);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_set_conf", rslt);
        return rslt;
    }


    heatr_conf->enable = BME68X_ENABLE;
    heatr_conf->heatr_temp = sensor_settings->heater_temperature;
    heatr_conf->heatr_temp_prof = sensor_settings->heater_temperature_profile;
    heatr_conf->heatr_dur = sensor_settings->heater_duration;
    heatr_conf->heatr_dur_prof = sensor_settings->heater_duration_profile;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, heatr_conf, bme);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
        return rslt;
    }

    rslt = bme68x_set_op_mode(sensor_settings->op_mode, bme);
    if (rslt != BME68X_OK)
    {
        bme68x_check_rslt("bme68x_set_op_mode", rslt);
        return rslt;
    }

    return rslt;
}

int8_t iot_bsec_init() {
    int8_t rslt;

    rslt = bsec_init();
    if (rslt != BSEC_OK)
    {
        fprintf(stderr, "bsec_init Error [%d]", rslt);
        return rslt;
    }

    /* Call to the function which sets the library with subscription information */
    rslt = bme680_bsec_update_subscription(BSEC_SAMPLE_RATE_LP);
    if (rslt != BSEC_OK)
    {
        fprintf(stderr, "bme680_bsec_update_subscription Error [%d]", rslt);
        return rslt;
    }

    return rslt;
}

int main()
{
    i2cOpen();
    i2cSetAddress(i2c_address);

    struct bme68x_dev bme;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;

    bsec_bme_settings_t sensor_settings;
    memset(&sensor_settings, 0, sizeof(sensor_settings));

    MQTTClient client;
    MQTTClient_deliveryToken token;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

    struct iot_processed_data processed_data;

    int8_t rslt;

    uint8_t n_fields;
    int64_t current_time_ns;
    uint32_t del_period;

    int rc;
    MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    rslt = iot_init_bme68x(&bme, &conf, &heatr_conf);
    if (rslt != BME68X_OK)
    {
        return rslt;
    }

    rslt = iot_bsec_init();
    if (rslt != BSEC_OK)
    {
        return rslt;
    }

    while (1)
    {
        current_time_ns = get_current_time_ns();
        rslt = bsec_sensor_control(current_time_ns, &sensor_settings);
        if (rslt != BSEC_OK)
        {
            fprintf(stderr, "bsec_sensor_control Error [%d]", rslt);
            return rslt;
        }

        if (sensor_settings.trigger_measurement)
        {
            rslt = iot_update_bme68x_config(&bme, &conf, &heatr_conf, &sensor_settings);                
            if (rslt != BME68X_OK)
            {
                return rslt;
            }

            // /* Calculate delay period in microseconds */
            del_period = bme68x_get_meas_dur(sensor_settings.op_mode, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
            bme.delay_us(del_period, bme.intf_ptr);

            /* Check if rslt == BME68X_OK, report or handle if otherwise */
            rslt = bme68x_get_data(sensor_settings.op_mode, &data, &n_fields, &bme);
            if (rslt != BME68X_OK)
            {
                bme68x_check_rslt("bme68x_get_data", rslt);
                return rslt;
            }

            if (n_fields)
            {

                rslt = iot_bsec_process_data(current_time_ns, &sensor_settings, &data, &processed_data);
                if (rslt != BSEC_OK)
                {
                    fprintf(stderr, "bsec_process_data Error [%d]", rslt);
                    return rslt;
                }

                printf(".");

                MQTTClient_message pubmsg = MQTTClient_message_initializer;
                char buffer[1024];
                int j;

                j = sprintf(buffer, "{");
                j += sprintf(buffer+j, "\"ts\":%lld", processed_data.timestamp/1000000);
                j += sprintf(buffer+j, ",\"temp\":%.2f", processed_data.temperature);
                j += sprintf(buffer+j, ",\"pressure\":%.2f", processed_data.raw_pressure);
                j += sprintf(buffer+j, ",\"humidity\":%.2f", processed_data.humidity);
                j += sprintf(buffer+j, ",\"raw_gas\":%.2f", processed_data.raw_gas);

                j += sprintf(buffer+j, ",\"co2_equivalent\":%.2f", processed_data.co2_equivalent);
                j += sprintf(buffer+j, ",\"co2_accuracy\":%d", processed_data.co2_accuracy);
                j += sprintf(buffer+j, ",\"iaq\":%.2f", processed_data.iaq);
                j += sprintf(buffer+j, ",\"iaq_accuracy\":%d", processed_data.iaq_accuracy);
                j += sprintf(buffer+j, ",\"static_iaq\":%.2f", processed_data.static_iaq);
                j += sprintf(buffer+j, ",\"static_iaq_accuracy\":%d", processed_data.static_iaq_accuracy);
                j += sprintf(buffer+j, ",\"breath_voc_equivalent\":%.2f", processed_data.breath_voc_equivalent);
                j += sprintf(buffer+j, ",\"breath_voc_accuracy\":%d", processed_data.breath_voc_accuracy);
                j += sprintf(buffer+j, ",\"status\":%d", rslt);
                j += sprintf(buffer+j, "}");

                char msg[j];
                sprintf(msg, "%s", buffer);

                pubmsg.payload = msg;
                pubmsg.payloadlen = strlen(msg);
                pubmsg.qos = QOS;
                pubmsg.retained = 0;
                MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);

                if ((rc = MQTTClient_waitForCompletion(client, token, TIMEOUT) != MQTTCLIENT_SUCCESS))
                {
                    printf("Failed to connect, return code %d\n", rc);
                    exit(EXIT_FAILURE);
                }
            }

        }

        bme68x_delay_us((sensor_settings.next_call - get_current_time_ns()) / 1000, NULL);
    }

    i2cClose();
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);

    return 0;
}