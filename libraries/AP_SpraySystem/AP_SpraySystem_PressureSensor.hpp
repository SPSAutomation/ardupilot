#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

#define PRESSURE_SENSOR_PACKET_SIZE_BYTES   4
#define PRESSURE_SENSOR_I2C_ADDRESS         (0x28 << 1)

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
     * @brief Get the last pressure psi value read from the sensor
     *
     * @return pressure value in psi
     */
    uint32_t get_pressure_psi();

    /**
     * @brief Get the last pressure bar value read from the sensor
     *
     * @return pressure value in bar
     */
    float get_pressure_bar();

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
    /* Flag to check whether the sensor is actually connected */
    bool device_connected{false};

    uint32_t last_read_pressure_psi{0};
    uint32_t last_read_temperature_c{0};

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> pressure_device;
};


#endif
