#pragma once

/**
 * This module keeps track of the amount of fluid that has flowed through the flow sensor. It also allows an external
 * task to calculate the flow rate by incrementing the flow time, and occasionally calling calculate_flow_rate_ml_min().
 * The flowed amount can be reset externally if the flow should not be recorded.
 */

#include "rolling_buffer.h"
#include "stdint.h"
#include "string.h"

/**
* I found that the factor from the manufacturer was consistently off by about 12%, and adjusted this constant by that,
* and got extremely consistent results. ANNOYINGLY, this learned factor is wrong when doing pulsing using the nozzle,
* but is perfectly correct when we just go full nozzle open ... This points to the opening/closing delays being wrong
* potentially. For MVP just leaving nozzle open when spraying is fine, this needs some time on it to tune these numbers
* in the future.
*
* Decent chance that this factor changes depending on its environment, e.g. test rig vs drone, so may need to be tuned.
*
* Original factor:  #define FLOW_SENSE_UL_PER_PULSE 145  // 1 rotation = 1 pulse = 144.99uL  https://docs.rs-online.com/7c18/0900766b8023e8dd.pdf
*/

#define FLOW_SENSE_UL_PER_PULSE 145 // This is specific to the GEMS 173936-C flow sensor
#define PULSE_TIME_TO_FLOW_ML_MIN 60000.0F

#define FLOW_RATE_ACCEPTABLE_MIN 1000  // ml/min
#define FLOW_RATE_ACCEPTABLE_MAX 6000  // ml/min

#define FLOW_RATE_DATA_BUF_SIZE 5

#ifdef __cplusplus
extern "C" {
#endif

#include "helpers.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/**
* Increment the number of pulses the flow sensor has seen
* User needs to set up interrupt to call this function when flow sensor triggers
*/
void increment_flow_sensor_pulse(uint32_t time_us);

/**
 * Enable or disable the sensor, tells the sensor that it should or shouldn't save pulses
 * @param value If the sensor should be treated as enabled or not
 */
void set_flow_sensor_enabled(bool value, uint64_t timestamp);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class FlowSensor {
private:

    float flow_rate_data[FLOW_RATE_DATA_BUF_SIZE];

    SemaphoreHandle_t mutex_flow_rate;

    /**
     *  The current flow rate as it is calculated each iteration - NOT buffered / filtered
     */
    float instant_flow_rate_ml_min{0};

    /*
     * If the sensor can be treated as "constantly open/enabled" - overrides the enabled flags
     */
    bool wide_open{false};

    /*
     * If the flow sensor is "enabled" - if data should be saved / we should listen to the sensor
     */
    bool enabled{true};
    uint64_t timestamp_enabled{0};
    uint64_t timestamp_disabled{0};

    uint64_t last_pulse_rising_edge_timstamp{0};

    /*
     * How much fluid passes per pulse of the sensor
     */
    uint16_t ul_per_pulse{FLOW_SENSE_UL_PER_PULSE};

    /*
     * Track time at which pulses are received
     */
    uint32_t last_pulse_time_us{0};

    uint16_t time_flow_ms{0};
    uint16_t total_time_flow_ms{0};
    uint32_t flow_amount_ul{0};
    uint16_t closing_delay_ms{0};
    float prev_amount_ml{0};

public:

    explicit FlowSensor() : flow_rate_rolling_buffer(flow_rate_data, FLOW_RATE_DATA_BUF_SIZE, 0)
    {
        mutex_flow_rate = xSemaphoreCreateMutex();
        memset(flow_rate_data, 0, FLOW_RATE_DATA_BUF_SIZE * sizeof(uint16_t));
    };

    /**
     * When a pulse is detected from the flow sensor, the time since the last pulse is stored in this buffer.
     * Calculation of the flow rate is then done at a later time based on the rolling average of these values.
     */
    rolling_buffer<float> flow_rate_rolling_buffer;

    /**
     * The number of pulses the flow sensor has seen, keeps track of amount that has flowed
     */
    uint16_t sensor_triggers_count{0};

    /**
     * Link a reference to the flow sensor to a static pointer
     * @param flow_sensor a pointer to the flow sensor instance
     */
    void flow_sensor_link_local(FlowSensor* flow_sensor);

    /**
     * Getters / setters
     */
    uint16_t get_instant_flow_rate_ml();
    uint16_t get_flow_rate_ml();
    uint32_t get_flow_amount_ul();
    float get_flow_amount_ml();
    uint16_t get_time_flow_ms();
    uint16_t get_total_time_flow_ms();
    void set_ul_per_pulse(uint16_t value);
    uint16_t get_ul_per_pulse();
    void set_wide_open(bool value);
    void set_enabled(bool value, uint64_t timestamp);
    bool is_enabled();
    void set_closing_delay_ms(uint16_t delay_ms);

    /**
     * Reset the flow sensor data to its initial state
     */
    void reset();
    void reset_flow_amount();

    /**
     * Increment the time that liquid has been flowing, used for flow rate calculations and reset when a calculation is done
     * Called by an external function / task / loop
     * @param time_ms the additional time that liquid has been flowing
     */
    void increment_time_flow(uint16_t time_ms);

    /**
     * Calculate the flow rate using the most recent data
     */
    void calculate_flow_rate();

    /**
     * Increment the number of pulses the flow sensor has seen
     */
    void increment_flow_sensor_pulse(uint32_t time_us);

};

void set_flow_sensor_instance(FlowSensor *flow_sensor);

FlowSensor* get_flow_sensor();

float* get_flow_sensor_pulses_buffer();

/**
 * Calculate a flow rate based on a amount and time
 * @param amount_ml amount in mls
 * @param time_ms time in ms
 * @return the flow rate for the given amount over the time
 */
uint16_t calculate_flow_rate_ml_min(float amount_ml, uint16_t time_ms);

#endif // __cplusplus
