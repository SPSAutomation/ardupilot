#include "AP_SpraySystem_PressureSensor.hpp"

extern const AP_HAL::HAL& hal;

AP_SpraySystem_PressureSensor::AP_SpraySystem_PressureSensor(uint8_t i2c_bus)
{
    /* Try to find the connected pressure sensor */
    pressure_device = std::move(hal.i2c_mgr->get_device(i2c_bus, PRESSURE_SENSOR_I2C_ADDRESS));

    if (pressure_device) {
        /* Device was found */
        device_connected = true;
    }

    pressure_device->set_retries(READ_RETRY_LIMIT);
}

void AP_SpraySystem_PressureSensor::update()
{
    uint8_t rx_buffer[PRESSURE_SENSOR_PACKET_SIZE_BYTES];
    uint16_t raw_value;
    uint16_t raw_temp_data;
    uint16_t raw_pressure_data;

    /* I2C transfers require getting the semaphore for the device first */
    WITH_SEMAPHORE(pressure_device->get_semaphore());

    /* Attempt to read out data from sensor */
    if (!pressure_device->transfer(nullptr, 0, rx_buffer, PRESSURE_SENSOR_PACKET_SIZE_BYTES))
    {
        /* Failed to read device */
        device_connected = false;
        return;
    }

    /* Read was successful, set connected flag */
    device_connected = true;

    /* Parse out data */
    raw_value = (rx_buffer[0] << 8) | rx_buffer[1];
    last_read_status = static_cast<AP_SpraySystem_PressureSensor_Status>((raw_value >> 14) & 0x03);

    /* Check status */
    if (last_read_status != AP_SpraySystem_PressureSensor_Status::OK)
    {
        /* Either a fault has occurred with the sensor, or the data is stale */
        return;
    }

    /* Parse out values */
    raw_pressure_data = raw_value & 0x3FFF;

    /* Get temperature data */
    raw_temp_data = static_cast<uint32_t>(((rx_buffer[2] << 3) | (rx_buffer[3] >> 5)) & 0x07FF);

    last_read_pressure_mbar = get_converted_pressure_value_mbar(raw_pressure_data);
    last_read_temperature_c = get_converted_temperature_value_c(raw_temp_data);
}

uint32_t AP_SpraySystem_PressureSensor::get_pressure_mbar()
{
    return last_read_pressure_mbar;
}

float AP_SpraySystem_PressureSensor::get_temperature_c()
{
    return last_read_temperature_c;
}

bool AP_SpraySystem_PressureSensor::sensor_connected()
{
    return device_connected;
}

AP_SpraySystem_PressureSensor_Status AP_SpraySystem_PressureSensor::get_current_status()
{
    return last_read_status;
}

uint32_t AP_SpraySystem_PressureSensor::get_converted_pressure_value_mbar(uint16_t raw_pressure_data)
{
    // See datasheet for this formula - https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MSP300&DocType=Data+Sheet&DocLang=English
    if (raw_pressure_data <= PRESSURE_DATA_OFFSET)
    {
        return 0;
    }

    uint16_t output = (raw_pressure_data - PRESSURE_DATA_OFFSET) * PRESSURE_DATA_MULTIPLIER;

    output *= PSI_TO_MBAR;

    return output;
}

float AP_SpraySystem_PressureSensor::get_converted_temperature_value_c(uint16_t raw_temp_data)
{
    /* Formula given by datasheet gives degrees C * 10 */
    float output = static_cast<float>(raw_temp_data * TEMP_DATA_MULTIPLIER - TEMP_DATA_OFFSET) / 10.0f;
    return output;
}