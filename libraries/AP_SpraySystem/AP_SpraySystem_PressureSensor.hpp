#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

#define PRESSURE_SENSOR_PACKET_SIZE_BYTES   4
#define PRESSURE_SENSOR_I2C_ADDRESS         (0x28)
#define READ_RETRY_LIMIT                    3

#define PRESSURE_SENSOR_MIN_MBAR 0
#define PRESSURE_SENSOR_MAX_MBAR 6894

#define PRESSURE_DATA_MULTIPLIER    (PRESSURE_SENSOR_MAX_MBAR - PRESSURE_SENSOR_MIN_MBAR) / (14000)
#define PRESSURE_DATA_OFFSET        1000

#define TEMP_DATA_MULTIPLIER (200 / 2048)
#define TEMP_DATA_OFFSET 50

/**
 * This class provides a driver for the I2C temperature and pressure sensor
 * used by the BFD spray system.
 */
class AP_SpraySystem_PressureSensor
{
public:
    AP_SpraySystem_PressureSensor(uint8_t i2c_bus);
    ~AP_SpraySystem_PressureSensor() = default;

    /**
     * @brief Update function to be called regularly.
     * Retrieves the pressure value from the sensor
     */
    void update();

    /**
     * @brief Get the last pressure bar value read from the sensor
     *
     * @return pressure value in bar
     */
    uint32_t get_pressure_mbar();

    /**
     * Get the last read temperature value in degrees celsius
     */
    uint32_t get_temperature_c();

    /**
     * @brief Returns whether the sensor is currently connected
     *
     * @return true if sensor is connected, false otherwise
     */
    bool sensor_connected();

private:

    /**
     * @brief Converts the raw numerical value provided by the sensor into
     * a readable mBar value
     *
     * @return pressure value in mBar
     */
    uint32_t get_converted_pressure_value_mbar(uint16_t raw_pressure_data);

    /**
     * @brief Converts the raw numerical value provided by the sensor into a
     * readable temperature value in degrees c
     */
    uint32_t get_converted_temperature_value_c(uint16_t raw_temp_data);

    /* Flag to check whether the sensor is actually connected */
    bool device_connected{false};

    /* Holders for calculated values */
    uint32_t last_read_pressure_mbar{0};
    uint32_t last_read_temperature_c{0};

    /* I2C device provided by the AP device manager */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> pressure_device;
};


#endif
