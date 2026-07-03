#pragma once

#include <Filter/AverageFilter.h>
#include <hal.h>
#include <AP_HAL_ChibiOS/SoftSigReaderInt.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
#include "stdint.h"
#include "string.h"
#include <AP_Param/AP_Param.h>

/* This is specific to the GEMS 173936-C flow sensor,
 * which is the default sensor used by the BFD spray system.
 * This setting is the default, but can be overridden by parameter configuration */
#define FLOW_SENSE_UL_PER_PULSE 145

#define PULSE_TIME_TO_FLOW_ML_MIN 60000.0F

#define FLOW_SENSOR_PULSE_DEBOUNCE_TIME_US  60

/* Keep a rolling average of 5 samples for the flow rate buffer
 * to mitigate jitter in the flow sensor pulse timing */
#define FLOW_RATE_DATA_BUF_SIZE 6

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief This class provides driver support for pulse-counter based turbine flow sensors.
 */
class AP_SpraySystem_FlowSensor
{
public:
    explicit AP_SpraySystem_FlowSensor() = default;

    /**
     * @brief Initialises the flow sensor driver with given EICU driver
     * and channel as well as flow sensor volume per pulse configuration
     */
    void init(EICUDriver *icu_drv, eicuchannel_t channel, eicuchannel_t aux_channel, float pulse_ul, uint32_t debounce_time_us);

    /**
     * @brief Link a reference to the flow sensor to a static pointer
     * @param flow_sensor a pointer to the flow sensor instance
     */
    void flow_sensor_link_local(AP_SpraySystem_FlowSensor* flow_sensor);

    /**
     * @brief Gets the last non-averaged flow rate detected by the sensor.
     *
     * @return last detected flow rate in ml / minute
     */
    uint16_t get_instant_flow_rate_ml();

    /**
     * @brief Gets the current filtered flow rate. This is preferred
     * for general usage as it provides a generally more stable value
     * that is more representative of the actual flow rate than the instantaneous
     * flow rate value
     *
     * @return current filtered flow rate in ml / minute
     */
    uint16_t get_flow_rate_ml();

    /**
     * @brief Gets the total flow volume in ul detected by the flow sensor since
     * the last flow volume reset.
     *
     * @return total flow volume in ul
     */
    uint32_t get_flow_amount_ul();

    /**
     * @brief Gets the total flow volume in ml since the last flow volume reset.
     *
     * @return total flow volume in ml
     */
    float get_flow_amount_ml();

    /**
     * @brief Sets the flow volume per pulse used to calculate the flow rate
     * and volume.
     *
     * @param value ul per pulse of the flow sensor to be used
     */
    void set_ul_per_pulse(float value);

    /**
     * @brief Get the cuyrrent configuration for flow sensor
     * volume per pulse
     */
    float get_ul_per_pulse();

    /**
     * @brief Enables or disables the EICU driver to turn
     * pulse counting on or off
     *
     * @param enabled true to enable pulse counting, false to disable
     */
    void set_enabled(bool enabled);

    /**
     * @brief reads the current state of the EICU driver and determines
     * whether the driver is enabled or disabled.
     *
     * @return true if currently enabled, false otherwise
     */
    bool is_enabled();

    /**
     * @brief Reset the flow sensor data to its initial state
     */
    void reset();

    /**
     * @brief Resets the current total volume tracked by the flow sensor
     */
    void reset_flow_amount();

    /**
     * @brief Increments the number of flow sensor pulses detected and calculates the
     * instantaneous and rolling average flow rate. This is generall called
     * from an ISR.
     *
     * @param eicup pointer to driver instance from which pulse times can be read
     */
    void increment_flow_sensor_pulse(EICUDriver *eicup);

private:

    /**
     *  The current flow rate as it is calculated each iteration - NOT buffered / filtered
     */
    float instant_flow_rate_ml_min{0};

    /*
     * How much fluid passes per pulse of the sensor
     */
    float ul_per_pulse{FLOW_SENSE_UL_PER_PULSE};

    /*
     * Track time at which pulses are received
     */
    uint32_t last_rising_edge_time{0};
    uint32_t last_falling_edge_time_us{0};

    /* Track the total flow volume */
    uint32_t flow_amount_ul{0};

    /* EICU driver used for accurate timestamping of pulses */
    EICUConfig icucfg;
    eicuchannel_t rising_edge_channel;
    eicuchannel_t falling_edge_channel;
    EICUChannelConfig channel_config;
    EICUChannelConfig aux_config;
    EICUDriver* _icu_drv = nullptr;

     /**
     * When a pulse is detected from the flow sensor, the time since the last pulse is stored in this buffer.
     * Calculation of the flow rate is then done at a later time based on the rolling average of these values.
     */
    AverageFilter<float, float, FLOW_RATE_DATA_BUF_SIZE> flow_rate_rolling_buffer;

    /**
     * Holder for current rolling average that can be read in a thread-safe manner.
     */
    float flow_rate_current_average{0};

    /**
     * The number of pulses the flow sensor has seen, keeps track of amount that has flowed
     */
    uint16_t sensor_triggers_count{0};
};

/**
 * @brief Sets the flow sensor instance to be called by the EICU IRQ
 */
void set_flow_sensor_instance(AP_SpraySystem_FlowSensor *flow_sensor);

/**
 * @brief IRQ callback
 */
void flow_sense_pulse_cb(EICUDriver *eicup, eicuchannel_t channel);

#endif // __cplusplus
