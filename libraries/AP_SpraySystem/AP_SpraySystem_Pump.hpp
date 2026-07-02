#pragma once

#include <hal.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

#define PUMP_MIN_THROTTLE_PERIOD 1100
#define PUMP_MAX_THROTTLE_PERIOD 1950
#define PUMP_OFF_THROTTLE_PERIOD 1050

/* 1MHz clock */
#define PWM_TIMER_CLOCK_HZ 1000000

/* 100 Hz */
#define PWM_TIMER_PERIOD_TICKS 5000

/* Pump speed to be used when agitating the spray tank */
#define PUMP_AGITATION_SPEED 1600

class AP_SpraySystem_Pump
{
public:
    AP_SpraySystem_Pump(PWMDriver *driver, uint8_t pwm_channel);
    ~AP_SpraySystem_Pump() = default;

    /**
     * @brief Turns on the pump with the currently configured speed
     */
    void enable();

    /**
     * @brief Turns off the pump. Pump speed setting is preserved.
     */
    void disable();

    /**
     * @brief Sets the current pump throttle value used by PWM control
     *
     * @param throttle_us throttle PWM width to be used
     *
     * @return true on success, false if throttle value is outside allowed range
     */
    bool set_speed(uint16_t throttle_us);

    /**
     * @brief Gets the current speed setting for the pump
     *
     * @return current pump throttle width in us
     */
    uint16_t get_speed();

    /**
     * @brief Checks whether the pump is currently running
     *
     * @return true if currently running, false otherwise
     */
    bool get_enabled();

private:
    /**
     * @brief Directly sets the PWM pulse width
     *
     * @param throttle pulse width to be used for throttle
     */
    inline void set_pwm_output(uint16_t throttle);

    /* Current throttle value to be used by the output PWM */
    uint16_t current_throttle_value;

    /* Is the pump currently running */
    bool enabled{false};

    uint8_t pump_pwm_channel;

    PWMDriver * pwm_driver;

    PWMConfig pwm_cfg;
};


#endif
