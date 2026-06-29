#include "AP_SpraySystem.hpp"

uint8_t flow_sensor_data[sizeof(AP_SpraySystem_FlowSensor)];
uint8_t spray_nozzle_data[sizeof(AP_SpraySystem_Nozzle)];
uint8_t return_line_data[sizeof(AP_SpraySystem_Nozzle)];
uint8_t pump_data[sizeof(AP_SpraySystem_Pump)];
uint8_t pressure_sensor_data[sizeof(AP_SpraySystem_PressureSensor)];

AP_SpraySystem *AP_SpraySystem::_singleton = nullptr;

extern const AP_HAL::HAL& hal;

AP_SpraySystem::AP_SpraySystem()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SpraySystem must be singleton");
    }
    _singleton = this;

    /* Initialise default parameters */
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_SpraySystem::init(void (*cb)(float, uint32_t, bool))
{
    /* Assign routine complete callback */
    routine_complete_cb = cb;

    /* Initialise flow sensor */
    flow_sensor = new(flow_sensor_data)AP_SpraySystem_FlowSensor();
    flow_sensor->init(&FLOW_SENSE_ICU_TIMER,FLOW_SENSE_ICU_CHANNEL, _flow_sense_pulse_ul);
    flow_sensor->set_enabled(true);

    /* Initialise spray nozzle */
    spray_nozzle = new(spray_nozzle_data)AP_SpraySystem_Nozzle(NOZZLE_CTRL_GPIO, NOZZLE_DUTY);
    return_line = new(return_line_data)AP_SpraySystem_Nozzle(RETURN_LINE_CTRL_GPIO, RETURN_LINE_DUTY);

    spray_nozzle->init();
    return_line->init();

    /* Initialise pump */
    pump = new(pump_data)AP_SpraySystem_Pump(&PUMP_PWM_DRIVER, PUMP_PWM_CHANNEL);

    /* Initialise pressure sensor */
    pressure_sensor = new(pressure_sensor_data)AP_SpraySystem_PressureSensor(PRESSURE_SENSOR_I2C_BUS);
}

void AP_SpraySystem::update()
{
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_time = 0;

    uint32_t dt_ms = now - last_update_time;

    if (dt_ms < FLOW_CONTROLLER_UPDATE_PERIOD_MS) {
        return;
    }

    /* Check for changes in the PID settings and update controller accordingly
     * Use a difference threshold because we are working with floats */
    if (fabsf(_p_gain - last_p_gain) > 0.0001) {
        pid_instance->set_kP(_p_gain);
        last_p_gain = _p_gain;
    }

    if (fabsf(_i_gain - last_i_gain) > 0.0001) {
        pid_instance->set_kI(_i_gain);
        last_i_gain = _i_gain;
    }

    if (fabsf(_d_gain - last_d_gain) > 0.0001) {
        pid_instance->set_kD(_d_gain);
        last_d_gain = _d_gain;
    }

    /* Update all sensors */
    pressure_sensor->update();

    switch (current_state)
    {
        case SpraySchedulerState::IDLE:
            /* If there is a spray routine queued, dequeue and schedule it */
            if (spray_routine_queue.available() > 0)
            {
                schedule_next_spray_routine();
            }
            break;

        case SpraySchedulerState::SCHEDULED:
            if (time_to_start_routine())
            {
                start_routine();
            }
            break;

        case SpraySchedulerState::RUNNING:
            flow_pid_step(dt_ms);
            break;
    }
}

SprayScheduleResult AP_SpraySystem::enqueue_spray_routine(SprayRoutine routine)
{
    /* Add the routine to the queue if there is room. */
    if (spray_routine_queue.space() == 0)
    {
        return SprayScheduleResult::QUEUE_FULL;
    }

    if (spray_routine_queue.push(routine))
    {
        return SprayScheduleResult::SUCCESS;
    }

    return SprayScheduleResult::QUEUE_FULL;
}

SprayScheduleResult AP_SpraySystem::schedule_next_spray_routine()
{
    /* Only allow scheduling while idle */
    if (current_state != SpraySchedulerState::IDLE)
    {
        return SprayScheduleResult::NOT_READY;
    }

    /* Get the next queued spray routine */
    if (!spray_routine_queue.pop(current_spray_routine))
    {
        return SprayScheduleResult::QUEUE_EMPTY;
    }

    /* Reset PID */
    pid_instance->reset_filter();

    /* Ensure pump is at it's agitation speed */
    return_line->open();
    pump->set_speed(PUMP_AGITATION_SPEED);
    pump->enable();

    // Set pump rate for this desired flow rate
    /* Enter SCHEDULED state */
    current_state = SpraySchedulerState::SCHEDULED;

    return SprayScheduleResult::SUCCESS;
}

bool AP_SpraySystem::time_to_start_routine()
{
    if (current_state != SpraySchedulerState::SCHEDULED)
    {
        /* No spray routine is currently scheduled */
        return false;
    }

    if (current_spray_routine.start_time_ms == 0)
    {
        /* Spray routine is not loaded */
        return false;
    }

    if ((get_current_time_millis() >= current_spray_routine.start_time_ms) && current_spray_routine.start_time_ms > 0)
    {
        return true;
    }
    return false;
}

void AP_SpraySystem::start_routine()
{
    current_state = SpraySchedulerState::RUNNING;
    flow_sensor->reset();
    return_line->close();
    spray_nozzle->open();
    flow_sensor->reset();
    flow_sensor->set_enabled(true);
}

void AP_SpraySystem::end_routine()
{
    /* Return to agitation */
    return_line->open();
    spray_nozzle->close();
    pump->set_speed(PUMP_AGITATION_SPEED);

    /* Tell the flow sensor to stop reading since the nozzle is now closed */
    flow_sensor->set_enabled(false);

    double amount_thresh = current_spray_routine.desired_spray_ml * AMOUNT_THRESHOLD_PROPORTION;

    bool routine_within_threshold = false;

    if (fabs(current_spray_routine.desired_spray_ml - flow_sensor->get_flow_amount_ml()) < amount_thresh)
    {
        routine_within_threshold = true;
    }

    if (routine_complete_cb != nullptr)
    {
        routine_complete_cb(flow_sensor->get_flow_amount_ml(),
                            time_spraying_ms,
                            routine_within_threshold);
    }

    time_spraying_ms = 0;
    flow_sensor->reset();
    current_state = SpraySchedulerState::IDLE;
}

void AP_SpraySystem::flow_pid_step(uint32_t dt_ms)
{
    time_spraying_ms += dt_ms;

    // Done spraying, tidy up, run PID controller
    if (time_spraying_ms >= current_spray_routine.time_allowed_ms)
    {
        if (flow_sensor->is_enabled())
        {
            // Close it ASAP so we can wait our delay period to ensure we have collected all flow sensor data,
            // closed by default in end_routine()
            spray_nozzle->close();
            hal.scheduler->delay(spray_nozzle->get_closing_delay_ms());
        }

        end_routine();
    }
    else
    {
        // Use a PID to try to keep a constant flow rate;
        float correction = pid_instance->update_all(current_spray_routine.desired_flow_rate_ml_min,
                                                          flow_sensor->get_flow_rate_ml(),
                                                          dt_ms);

        uint32_t new_pump_speed_us = pump->get_speed() + static_cast<int32_t>(correction);

        constrain_float(new_pump_speed_us, PUMP_MIN_THROTTLE_PERIOD, PUMP_MAX_THROTTLE_PERIOD);

        pump->set_speed(new_pump_speed_us);
    }
}

void AP_SpraySystem::set_time_offset(int64_t offset_us)
{
    montonic_clock_offset += (offset_us / 1000);
}

void AP_SpraySystem::set_spray_nozzle_open(bool open)
{
    (open) ? spray_nozzle->open() : spray_nozzle->close();
}

void AP_SpraySystem::set_return_line_open(bool open)
{
    (open) ? return_line->open() : return_line->close();
}

bool AP_SpraySystem::set_pump_speed(uint32_t pump_throttle_value)
{
    return pump->set_speed(pump_throttle_value);
}

void AP_SpraySystem::set_pump_enabled(bool enabled)
{
    (enabled) ? pump->enable() : pump->disable();
}

uint64_t AP_SpraySystem::get_current_time_millis()
{
    return AP_HAL::millis64() + montonic_clock_offset;
}

uint32_t AP_SpraySystem::get_current_flow_rate_ml_min() {
    return flow_sensor->get_flow_rate_ml();
}

uint32_t AP_SpraySystem::get_current_pressure_mbar()
{
    return pressure_sensor->get_pressure_mbar();
}

float AP_SpraySystem::get_current_temperature_c()
{
    return pressure_sensor->get_temperature_c();
}

uint32_t AP_SpraySystem::get_current_spray_time_ms()
{
    return time_spraying_ms;
}

uint8_t AP_SpraySystem::get_spray_nozzle_state()
{
    return (spray_nozzle->is_open() ? 1 : 0);
}

float AP_SpraySystem::get_amount_flowed_ml()
{
    return flow_sensor->get_flow_amount_ml();
}

uint16_t AP_SpraySystem::get_current_pump_speed()
{
    return pump->get_speed();
}

bool AP_SpraySystem::get_pump_enabled()
{
    return pump->get_enabled();
}

/* Parameters */
const AP_Param::GroupInfo AP_SpraySystem::var_info[] = {
        // @Param: FLOW_SENSE_PULSE_UL
        // @DisplayName: Flow Pulse ul
        // @Description: Number of ul per flow sensor pulse
        // @User Standard
        // @RebootRequired: True
        AP_GROUPINFO("PULSE_UL", 1, AP_SpraySystem, _flow_sense_pulse_ul, FLOW_SENSE_UL_PER_PULSE),

        // @Param: FLOW_P_GAIN
        // @DisplayName: Flow Controller P Gain
        // @Description: P gain for flow controller PID
        // @User Standard
        // @RebootRequired: False
        AP_GROUPINFO("P_GAIN", 2, AP_SpraySystem, _p_gain, 0.8),

        // @Param: FLOW_I_GAIN
        // @DisplayName: Flow Controller I Gain
        // @Description: I gain for flow controller PID
        // @User Standard
        // @RebootRequired: False
        AP_GROUPINFO("I_GAIN", 3, AP_SpraySystem, _i_gain, 0.1),

        // @Param: FLOW_D_GAIN
        // @DisplayName: Flow Controller D Gain
        // @Description: D gain for flow controller PID
        // @User Standard
        // @RebootRequired: False
        AP_GROUPINFO("P_GAIN", 4, AP_SpraySystem, _d_gain, 0.2),

        AP_GROUPEND
};