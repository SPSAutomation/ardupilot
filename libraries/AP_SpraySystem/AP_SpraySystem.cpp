#include "AP_SpraySystem.hpp"

uint8_t flow_sensor_data[sizeof(AP_SpraySystem_FlowSensor)];
uint8_t spray_nozzle_data[sizeof(AP_SpraySystem_Nozzle)];
uint8_t pump_data[sizeof(AP_SpraySystem_Pump)];
uint8_t pressure_sensor_data[sizeof(AP_SpraySystem_PressureSensor)];

AP_SpraySystem *AP_SpraySystem::_singleton = nullptr;

AP_SpraySystem::AP_SpraySystem()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_SpraySystem must be singleton");
    }
    _singleton = this;
}

void AP_SpraySystem::init()
{
    /* Initialise default parameters */
    AP_Param::setup_object_defaults(this, var_info);

    /* Initialise flow sensor */
    flow_sensor = new(flow_sensor_data)AP_SpraySystem_FlowSensor();
    flow_sensor->init(&FLOW_SENSE_ICU_TIMER,FLOW_SENSE_ICU_CHANNEL, _flow_sense_pulse_ul);
    flow_sensor->set_enabled(true);

    /* Initialise spray nozzle */
    spray_nozzle = new(spray_nozzle_data)AP_SpraySystem_Nozzle(6, 50);

    /* Initialise pump */
    pump = new(pump_data)AP_SpraySystem_Pump(&PWMD3, 0);

    /* Initialise pressure sensor */
    pressure_sensor = new(pressure_sensor_data)AP_SpraySystem_PressureSensor(0);
}

void AP_SpraySystem::update()
{
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_time = 0;

    uint32_t dt = now - last_update_time;

    if (dt < FLOW_CONTROLLER_UPDATE_PERIOD_MS) {
        return;
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
            iterate_flow_control(dt_ms);
            break;
    }
}

SprayScheduleResult AP_SpraySystem::enqueue_spray_routine(SprayRoutine routine)
{
    SprayRoutine routine;

    // Set the desired amounts, rate, amount, time
    desired_flow_rate_ml_min = rate_ml_min;
    time_allowed_ms = time_ms;
    time_spraying_ms = 0;

    /* Add the routine to the queue if there is room. */
    if (spray_routine_queue.is_full())
    {
        return SprayScheduleResult::QUEUE_FULL;
    }

    /* If a routine is not currently scheduled, set the pump speed to prepare */
    routine.desired_flow_rate_ml_min = desired_flow_rate_ml_min;
    routine.desired_spray_ml = amount_ml;
    routine.time_allowed_ms = time_allowed_ms;
    routine.time_spraying_ms = time_spraying_ms;
    routine.start_time_ms = start_time_utc_ms;

    if (spray_routine_queue.enqueue(routine))
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
    if (!spray_routine_queue.dequeue(&current_spray_routine))
    {
        return SprayScheduleResult::QUEUE_EMPTY;
    }

    /* Reset PID */
    pid_instance->reset_filter();

    /* Ensure pump is at it's agitation speed */
    pump_speed_us = PUMP_AGITATION_SPEED;
    return_line->open();
    pump->set_speed(PUMP_AGITATION_SPEED);
    pump->enable();

    // Set pump rate for this desired flow rate
    /* Enter SCHEDULED state */
    current_state = SpraySchedulerState::SCHEDULED;

    return SprayScheduleResult::SUCCESS;
}

SprayScheduleResult AP_SpraySystem::time_to_start_routine()
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
    last_desired_flow_rate_ml_min = current_spray_routine.desired_flow_rate_ml_min;
    flow_sensor->reset();
    current_state = SpraySchedulerState::IDLE;
}

void AP_SpraySystem::flow_pid_step(uint32_t dt_ms)
{
    time_spraying_ms += dt_ms;
    time_since_last_adjustment += dt_ms;

    // Attempt to compensate for the extra liquid due to the delay of closing the nozzle - this is only ever going to be ~ 1 - 4 ml
    double estimated_flow_closing = 0;
    if (flow_sensor->is_enabled())
    {
        estimated_flow_closing = spray_nozzle->get_closing_delay_ms() * ((float) flow_sensor->get_flow_rate_ml() / (60 * 1000.0f));
    }

    // Done spraying, tidy up, run PID controller
    if (time_spraying_ms >= current_spray_routine.time_allowed_ms)
    {

        double actual_flow_closing = flow_sensor->get_flow_amount_ml();
        if (flow_sensor->is_enabled())
        {
            // Close it ASAP so we can wait our delay period to ensure we have collected all flow sensor data,
            // closed by default in end_routine()
            spray_nozzle->close();

        }
        actual_flow_closing = flow_sensor->get_flow_amount_ml() - actual_flow_closing;

        end_routine();
    }
    else
    {
        if (time_since_last_adjustment > PID_LOOP_PERIOD_MS)
        {
            // Use a PID to try to keep a constant flow rate
            double integral_time_iter = time_since_last_adjustment;

            pid_instance->update_all(current_spray_routine.desired_flow_rate_ml_min,
                                                              flow_sensor->get_flow_rate_ml(),
                                                              integral_time_iter);

            current_pump_speed_us *= pid_instance->get_p();
            current_pump_speed_us *= pid_instance->get_d();
            current_pump_speed_us *= pid_instance->get_i();

            pump->set_speed(current_pump_speed_us);

            time_since_last_adjustment = 0;
        }
    }
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

/* Parameters */
const AP_Param::GroupInfo AP_SpraySystem::var_info[] = {
        // @Param: FLOW_SENSE_PULSE_UL
        // @DisplayName: Flow Pulse ul
        // @Description: Number of ul per flow sensor pulse
        // @User Standard
        // @RebootRequired: True
        AP_GROUPINFO("PULSE_UL", 1, AP_SpraySystem, _flow_sense_pulse_ul, FLOW_SENSE_UL_PER_PULSE),

        AP_GROUPEND
};