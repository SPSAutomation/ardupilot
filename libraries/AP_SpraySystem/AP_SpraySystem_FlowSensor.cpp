#include "AP_SpraySystem_FlowSensor.hpp"

static AP_SpraySystem_FlowSensor* flow_sensor_instance = nullptr;

void AP_SpraySystem_FlowSensor::init(EICUDriver *icu_drv, eicuchannel_t channel, float pulse_ul)
{
    /* Set the local flow sensor instance to this */
    set_flow_sensor_instance(this);

    rising_edge_channel = channel;
    falling_edge_channel = aux_channel;

    /* Configure the ChibiOS EICU driver for accurate timestamping of flow pulses */
    _icu_drv = icu_drv;
    icucfg.dier = 0;
    icucfg.frequency = INPUT_CAPTURE_FREQUENCY;
    for (int i=0; i< EICU_CHANNEL_ENUM_END; i++) {
        icucfg.iccfgp[i]=nullptr;
    }

    icucfg.iccfgp[rising_edge_channel] = &channel_config;
    icucfg.iccfgp[falling_edge_channel] = &aux_config;

    /* Main pulse counting is performed on the rising edge, while
     * the falling edge is used for debouncing */
    channel_config.alvl = EICU_INPUT_ACTIVE_HIGH;
    channel_config.capture_cb = flow_sense_pulse_cb;
    aux_config.alvl = EICU_INPUT_ACTIVE_LOW;
    aux_config.capture_cb = nullptr;

    /* Start EICU */
    eicuStart(_icu_drv, &icucfg);

    //sets input filtering to 4 timer clock
    stm32_timer_set_input_filter(_icu_drv->tim, rising_edge_channel, 8);
    stm32_timer_set_input_filter(_icu_drv->tim, falling_edge_channel, 8);
    stm32_timer_set_channel_input(_icu_drv->tim, falling_edge_channel, 2);

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
    /* Reset flow amount */
    flow_amount_ul = 0;
    sensor_triggers_count = 0;

    /* Reset flow rates */
    instant_flow_rate_ml_min = 0;
    flow_rate_rolling_buffer.reset();
    flow_rate_current_average = 0;

    /* Reset last pulse time */
    last_rising_edge_time = 0;
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

void AP_SpraySystem_FlowSensor::set_ul_per_pulse(float value)
{
    ul_per_pulse = value;
}

float AP_SpraySystem_FlowSensor::get_ul_per_pulse()
{
    return ul_per_pulse;
}

void flow_sense_pulse_cb(EICUDriver *eicup, eicuchannel_t channel)
{
    if (flow_sensor_instance != nullptr) {
        flow_sensor_instance->increment_flow_sensor_pulse(eicup);
    }
}

void AP_SpraySystem_FlowSensor::increment_flow_sensor_pulse(EICUDriver *eicup)
{
    uint32_t pulse_time_us;
    float calculated_flow_rate_ml_min;

    uint32_t rising_timestamp_us = eicup->tim->CCR[rising_edge_channel];
    uint32_t falling_timestamp_us = eicup->tim->CCR[falling_edge_channel];

    uint32_t debounce_time_us = rising_timestamp_us - falling_timestamp_us;

    /* Run debouncing. If a previous falling edge was detected, this
     * rising edge should not have occurred for a minimum amount of time */
    if (debounce_time_us < FLOW_SENSOR_PULSE_DEBOUNCE_TIME_US)
    {
        /* Filter this pulse */
        return;
    }

    /* Calculate the time since the last pulse was detected */
    if (last_rising_edge_time != 0)
    {
        /* Because both time_us and last_pulse_time_us are both unsigned, this
         * operation is unaffected if the clock wraps around between this pulse and
         * the previous pulse */
        pulse_time_us = rising_timestamp_us - last_rising_edge_time;;

        /* Handle zero case */
        if (pulse_time_us == 0)
        {
            return;
        }
        calculated_flow_rate_ml_min = (ul_per_pulse * PULSE_TIME_TO_FLOW_ML_MIN) / pulse_time_us;

        flow_rate_current_average = flow_rate_rolling_buffer.apply(calculated_flow_rate_ml_min);
        instant_flow_rate_ml_min = calculated_flow_rate_ml_min;
    }

    // increment pulse
    flow_amount_ul += ul_per_pulse;
    sensor_triggers_count++;

    last_rising_edge_time = rising_timestamp_us;
}

void AP_SpraySystem_FlowSensor::set_enabled(bool value)
{
    if (value)
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
    switch (_icu_drv->state) {
        case eicustate_t::EICU_UNINIT:
        case eicustate_t::EICU_STOP:
            return false;

        default:
            /* All other states are active */
            return true;
    }
}

void set_flow_sensor_instance(AP_SpraySystem_FlowSensor* flow_sensor)
{
    flow_sensor_instance = flow_sensor;
}