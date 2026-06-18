#include "pwm_pump.hpp"
#include "helpers.h"

void Pump::initialise()
{
    state = PumpState::INACTIVE;

    PwmOutputResult res = pwm.initialise_pwm();
    if (res == PwmOutputResult::FAIL)
    {
        error_handler(PumpError::START_ERROR);
        return;
    }
    turn_off();

}

void Pump::stop_signal()
{
    // This actually stops the pwm signal, rather than setting the pump to 'off', which is just 1000us period
    state = PumpState::INACTIVE;

    PwmOutputResult res = pwm.stop_pwm_generation();
    if (res == PwmOutputResult::FAIL)
    {
        error_handler(PumpError::STOP_ERROR);
        return;
    }

}

void Pump::set_speed(uint16_t pwm_pulse_length_us)
{
    // Clamp pump speed - setting the pump speed should always result in
    // the pump moving to the active state.
    pwm_pulse_length_us = MAX(PWM_PUMP_MIN_US, pwm_pulse_length_us);
    pwm_pulse_length_us = MIN(PWM_PUMP_MAX_US, pwm_pulse_length_us);

    state = PumpState::ACTIVE;
    PwmOutputResult res = pwm.set_pwm_period_us(pwm_pulse_length_us);

    if (res == PwmOutputResult::FAIL)
    {
        error_handler(PumpError::STOP_ERROR);
    }
}

void Pump::turn_off()
{
    // Not using stop_pwm_pump_generation as we want to still put out a signal to the pump
    state = PumpState::INACTIVE;

    PwmOutputResult res = pwm.set_pwm_period_us(PWM_PUMP_OFF_US);

    if (res == PwmOutputResult::FAIL)
    {
        error_handler(PumpError::STOP_ERROR);
    }
}

bool Pump::is_active()
{
    return state == PumpState::ACTIVE;
}

PumpState Pump::get_state()
{
    return state;
}

void Pump::error_handler(PumpError error)
{

    send_debug_message("PWM pump error handler: %s", pump_error_to_string(error));

    switch (error)
    {
        case PumpError::START_ERROR:
            state = PumpState::ERROR;
            break;
        case PumpError::STOP_ERROR:
            state = PumpState::ERROR;
            break;
        case PumpError::INVALID_PERIOD:
            stop_signal();
            break;
    }
}

uint16_t Pump::get_current_period_us()
{
    return pwm.get_current_period_us();
}

const char* pump_error_to_string(PumpError error)
{
    switch (error)
    {
        case PumpError::START_ERROR:
            return "START_ERROR";
        case PumpError::STOP_ERROR:
            return "STOP_ERROR";
        case PumpError::INVALID_PERIOD:
            return "INVALID_PERIOD";
        default:
            return "UNKNOWN_ERROR";
    }
}