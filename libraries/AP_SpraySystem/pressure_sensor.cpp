#include "pressure_sensor.hpp"

bool PressureSensor::read_data()
{
    send_measurement_request();

    uint8_t raw_data[PRESSURE_SENSOR_PACKET_SIZE];
    I2CResult result = i2c.read(raw_data, PRESSURE_SENSOR_PACKET_SIZE);

    if (result == I2CResult::SUCCESS)
    {
        process_data(raw_data);
        return true;
    } else
    {
        read_data_error_count++;
        return false;
    }
}

void PressureSensor::process_data(uint8_t *raw_data)
{

    uint16_t raw_value = (raw_data[0] << 8) | raw_data[1];
    uint8_t status_bits = (raw_value >> 14) & 0x03;

    if (status_bits == 0b00) { // Valid data
        uint16_t bridge_data = raw_value & 0x3FFF; // Mask status bits
        uint16_t temperature_data = ((raw_data[2] << 3) | (raw_data[3] >> 5)) & 0x07FF;  // Mask last 5 bits

        uint16_t pressure_unbuf = convert_pressure_data(bridge_data);
        pressure_sense_buffer_psi.update_buf(pressure_unbuf);
        pressure_psi = pressure_sense_buffer_psi.get_buf_avg();
        temperature_c = convert_temperature_data(temperature_data);

        new_data = true;
    } else {
        // Failure
        read_data_error_count++;
    }
}

uint16_t PressureSensor::get_pressure_psi()
{
    new_data = false;
    return pressure_psi;
}

uint16_t PressureSensor::get_temperature_c()
{
    return temperature_c;
}

uint64_t PressureSensor::get_read_data_error_count()
{
    return read_data_error_count;
}

bool PressureSensor::is_new_data()
{
    return new_data;
}

void PressureSensor::send_measurement_request()
{
    uint8_t dummy_data;
    i2c.read(&dummy_data, 0);
    ms_sleep(2);
}

uint16_t PressureSensor::convert_pressure_data(uint16_t data)
{
    // See datasheet for this formula - https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MSP300&DocType=Data+Sheet&DocLang=English
    if (data <= 1000) return 0;

    uint16_t output = ((data - 1000) * (PRESSURE_SENSOR_MAX_PSI - PRESSURE_SENSOR_MIN_PSI)) / (15000 - 1000);
    output = clamp_int(output, PRESSURE_SENSOR_MIN_PSI, PRESSURE_SENSOR_MAX_PSI);
    return output;
}

uint16_t PressureSensor::convert_temperature_data(uint16_t data)
{
    // See datasheet for this formula - https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MSP300&DocType=Data+Sheet&DocLang=English
    uint16_t output = (((150 - -50) * data) / 2048) - 50;
    return output;
}
