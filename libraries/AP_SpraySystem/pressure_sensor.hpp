#pragma once

/**
 * This module handles taking readings from the pressure sensor. It reads pressure readings, and temperatures of the
 * medium being measured.
 * To actuate a read, read_data() must be called externally, then fetched using get_pressure_psi/get_temperature_c.
 */

#ifdef __cplusplus
extern "C" {
#endif
#include "helpers.h"
#ifdef __cplusplus
}
#endif

#include <cstring>
#include "i2c_abc.hpp"
#include "rolling_buffer.h"

#define PRESSURE_SENSOR_I2C_ADDRESS (0x28 << 1)  // 7 bit address, must be shifted

#define PRESSURE_SENSOR_MIN_PSI 0
#define PRESSURE_SENSOR_MAX_PSI 100

// See datasheet for other options, this is the full amount of data available
//   https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MSP300&DocType=Data+Sheet&DocLang=English
#define PRESSURE_SENSOR_PACKET_SIZE 4

#define PRESSURE_SENSOR_DATA_BUF_SIZE 5

class PressureSensor
{
private:

    uint16_t pressure_psi;
    uint16_t temperature_c;
    I2C& i2c;
    bool new_data;
    uint64_t read_data_error_count;

    uint16_t pressure_sense_data[PRESSURE_SENSOR_DATA_BUF_SIZE];

    /**
     * Tell the sensor to start a measurement cycle to ensure fresh data
     */
    void send_measurement_request();
    
public:

    explicit PressureSensor(I2C& i2c) : pressure_psi(0),
                                        temperature_c(0),
                                        i2c(i2c),
                                        new_data(false),
                                        read_data_error_count(0),
                                        pressure_sense_buffer_psi(pressure_sense_data, PRESSURE_SENSOR_DATA_BUF_SIZE, 65535) {
        // Zero fill
        memset(pressure_sense_data, 0, PRESSURE_SENSOR_DATA_BUF_SIZE * sizeof(uint16_t));
    }

    rolling_buffer<uint16_t> pressure_sense_buffer_psi;

    /**
     * Read data from the sensor, triggers a measurement cycle, then a read from the sensor.
     * @return bool, if the data read & process was successful - tells the user if there is fresh data
     */
    bool read_data();

    /**
     * Process the read data, if it is valid, assign the data.
     * @param raw_data The raw data read from the sensor
     */
    void process_data(uint8_t *raw_data);

    /**
     * Convert the processed pressure data into a real pressure
     * @param data the processed data to convert
     * @return the real pressure, in psi
     */
    static uint16_t convert_pressure_data(uint16_t data);

    /**
     * Convert the processed temperature data into a real temperature
     * @param data the processed data to convert
     * @return the real temperature, in degrees celsius
     */
    static uint16_t convert_temperature_data(uint16_t data);

    uint16_t get_pressure_psi();
    uint16_t get_temperature_c();
    uint64_t get_read_data_error_count();

    /**
     * Is there new data available from the sensor. Reset when data is accessed
     * @return bool, if there is new data
     */
    bool is_new_data();
};
