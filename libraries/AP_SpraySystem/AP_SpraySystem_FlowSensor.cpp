#include "AP_SpraySystem_FlowSensor.hpp"

static AP_SpraySystem_FlowSensor* flow_sensor_instance = nullptr;
AP_SpraySystem_FlowSensor::AP_SpraySystem_FlowSensor()
{
}

void AP_SpraySystem_FlowSensor::init(EICUDriver *icu_drv, eicuchannel_t channel, float pulse_ul)
{
    set_flow_sensor_instance(this);

    last_value = 0;
    _icu_drv = icu_drv;
    icucfg.dier = 0;
    icucfg.frequency = INPUT_CAPTURE_FREQUENCY;
    for (int i=0; i< EICU_CHANNEL_ENUM_END; i++) {
        icucfg.iccfgp[i]=nullptr;
    }

    icucfg.iccfgp[channel] = &channel_config;
    channel_config.alvl = EICU_INPUT_ACTIVE_LOW;
    channel_config.capture_cb = flow_sense_pulse_cb;

    eicuStart(_icu_drv, &icucfg);
    eicuEnable(_icu_drv);

    /* Apply flow sensor pulse volume configuration */
    set_ul_per_pulse(pulse_ul);
}

uint16_t AP_SpraySystem_FlowSensor::get_instant_flow_rate_ml()
{
    return instant_flow_rate_ml_min;
}

uint32_t AP_SpraySystem_FlowSensor::get_flow_amount_ul()
{
    /* Because this is incremented via interrupt, this must be a critical section */
    chSysLock();
    uint32_t amount = flow_amount_ul;
    chSysUnlock();
    return amount;
}

float AP_SpraySystem_FlowSensor::get_flow_amount_ml()
{
    float amount = ((float) get_flow_amount_ul()) / 1000.0f;
    return amount;
}

void AP_SpraySystem_FlowSensor::reset()
{
    flow_amount_ul = 0;
    prev_amount_ml = 0;
    time_flow_ms = 0;
    total_time_flow_ms = 0;
    instant_flow_rate_ml_min = 0;
    sensor_triggers_count = 0;
    flow_rate_rolling_buffer.reset();
    enabled = false;
    last_pulse_time_us = 0;
}

void AP_SpraySystem_FlowSensor::reset_flow_amount()
{
    flow_amount_ul = 0;
}

uint16_t AP_SpraySystem_FlowSensor::get_flow_rate_ml()
{
    chSysLock();
    float rate = flow_rate_current_average;
    chSysUnlock();
    return (uint16_t) rate;
}

uint16_t AP_SpraySystem_FlowSensor::get_time_flow_ms()
{
    return time_flow_ms;
}

uint16_t AP_SpraySystem_FlowSensor::get_total_time_flow_ms()
{
    return total_time_flow_ms;
}

void AP_SpraySystem_FlowSensor::set_ul_per_pulse(uint16_t value)
{
    ul_per_pulse = value;
}

uint16_t AP_SpraySystem_FlowSensor::get_ul_per_pulse()
{
    return ul_per_pulse;
}

void flow_sense_pulse_cb(EICUDriver *eicup, eicuchannel_t channel)
{
    if (flow_sensor_instance != nullptr) {
        flow_sensor_instance->increment_flow_sensor_pulse(eicup->tim->CCR[channel]);
    }
}

void AP_SpraySystem_FlowSensor::increment_flow_sensor_pulse(uint32_t time_us)
{
    uint32_t pulse_time_us;
    float calculated_flow_rate_ml_min;

    // increment pulse
    flow_amount_ul += ul_per_pulse;
    sensor_triggers_count++;

    /* Calculate the time since the last pulse was detected */
    if (last_pulse_time_us != 0)
    {
        pulse_time_us = time_us - last_pulse_time_us;
        calculated_flow_rate_ml_min = ul_per_pulse * PULSE_TIME_TO_FLOW_ML_MIN / pulse_time_us;

        flow_rate_current_average = flow_rate_rolling_buffer.apply(calculated_flow_rate_ml_min);
        instant_flow_rate_ml_min = calculated_flow_rate_ml_min;
    }

    last_pulse_time_us = time_us;
}

void AP_SpraySystem_FlowSensor::set_enabled(bool value, uint64_t timestamp)
{
    enabled = value;

    if (enabled)
    {
        eicuEnable(_icu_drv);
    }
    else
    {
        eicuDisable(_icu_drv);
    }
}

bool AP_SpraySystem_FlowSensor::is_enabled()
{
    return enabled;
}

void AP_SpraySystem_FlowSensor::increment_time_flow(uint16_t time_ms)
{
    time_flow_ms += time_ms;
    total_time_flow_ms += time_ms;
}

void set_flow_sensor_instance(AP_SpraySystem_FlowSensor* flow_sensor)
{
    flow_sensor_instance = flow_sensor;
}

void increment_flow_sensor_pulse(uint32_t time_us)
{
    // c wrapper for interrupt to access
    if (flow_sensor_instance)
    {
        flow_sensor_instance->increment_flow_sensor_pulse(time_us);
    }
}

void AP_SpraySystem_FlowSensor::set_closing_delay_ms(uint16_t delay_ms)
{
    closing_delay_ms = delay_ms;
}