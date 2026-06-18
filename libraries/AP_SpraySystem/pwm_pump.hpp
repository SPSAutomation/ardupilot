#pragma once

/**
 * This module controls a pump via a PWM signal. The underlying pump is a brushless DC motor with an integrated ESC.
 * The pump has a discrete resolution in 100us steps from 1000-2000.
 * This module uses a generic PwmOutput module to control its PWM signal.
 */

#include <stdarg.h>
#include <stdio.h>

#include "pwm_output_abc.hpp"
#include "uart_communication.hpp"

#define PWM_PUMP_DEFAULT_US 1900
#define PWM_PUMP_OFF_US 1000
#define PWM_PUMP_MIN_US 1400
#define PWM_PUMP_MAX_US 2000

#define PWM_PUMP_CONTROL_STEP_US 10

#define MICRO_SEC 1000000

enum class PumpState {
    INACTIVE,      // Inactive, pump not running
    ACTIVE,        // Active, running
    ERROR
};


enum class PumpError {
    START_ERROR,
    STOP_ERROR,
    INVALID_PERIOD
};

class Pump {
private:
    PwmOutput& pwm;
    PumpState state;

public:
    Pump(PwmOutput& pwm_output) : pwm(pwm_output), state(PumpState::INACTIVE)
    {
        initialise();
    }

    /**
     * Initialise the pump
     */
    void initialise();

    /**
     * Stop the signal to the pump being generated
     */
    void stop_signal();

    /**
     * Set the desired speed of the pump
     * @param pwm_pulse_length_us desired speed of the pump, in us pulse length
     */
    void set_speed(uint16_t pwm_pulse_length_us);

    /**
     * Turn off pump
     */
    void turn_off();

    /**
     * Is the pump active
     * @return bool, if the pump is active
     */
    bool is_active();

    PumpState get_state();
    uint16_t get_current_period_us();

    void error_handler(PumpError error);

};

const char* pump_error_to_string(PumpError error);
