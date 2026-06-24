#include "AP_SpraySystem_PressureSensor.hpp"

extern const AP_HAL::HAL& hal;

AP_SpraySystem_PressureSensor::AP_SpraySystem_PressureSensor(uint8_t i2c_bus)
{
    /* Try to find the connected pressure sensor */
    pressure_device = hal.i2c_mgr->get_device(i2c_bus, PRESSURE_SENSOR_I2C_ADDRESS);

    if (pressure_device) {
        /* Device was found */
        device_connected = true;
    }
}

void AP_SpraySystem_PressureSensor::update()
{
//    uint8_t rx_buffer[PRESSURE_SENSOR_PACKET_SIZE_BYTES];
//    uint16_t raw_value;
//    uint8_t status_bitmap;
//    uint16_t raw_pressure_temp_data;

    /* First, check that the device is present to be read */
    if (!device_connected)
    {
        return;
    }

//    /* Attempt to read out data from sensor */
//    if (!pressure_device.read_registers(0, rx_buffer, PRESSURE_SENSOR_PACKET_SIZE_BYTES))
//    {
//        /* Failed to read device */
//        return;
//    }
//
//    /* Parse out data */
//    raw_value = (rx_buffer[0] << 8) | rx_buffer[1];
//    status_bitmap = (raw_value >> 14) & 0x03;
//
//    /* Check status bits */
//    if (status_bitmap != 0b00)
//    {
//        return;
//    }
//
//    /* Parse out values */
//    raw_pressure_temp_date = raw_value & 0x3FFF;
//
//    /* Get temperature data */
//    last_read_temperature_c = static_cast<uint32_T>(((rx_buffer[2] << 3) | (rx_buffer[3] >> 5)) & 0x07FF)

}