#include "AP_SpraySystem_Pump.hpp"

AP_SpraySystem_Pump::AP_SpraySystem_Pump(PWMDriver *driver, uint8_t pwm_channel)
{
    pwm_driver = driver;
    pump_pwm_channel = pwm_channel;

    if (pwm_channel >= PWM_CHANNELS)
    {
        /* Channel out of bounds */
        return;
    }

    pwm_cfg.frequency = PWM_TIMER_CLOCK_HZ;
    pwm_cfg.period = PWM_TIMER_PERIOD_TICKS;
    pwm_cfg.callback = nullptr;

    /* Only apply the active configuration to the channel we are using */
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (i == pwm_channel)
        {
            pwm_cfg.channels[i].mode = PWM_OUTPUT_ACTIVE_HIGH;
            pwm_cfg.channels[i].callback = nullptr;
        }
        else
        {
            pwm_cfg.channels[i].mode = PWM_OUTPUT_DISABLED;
            pwm_cfg.channels[i].callback = nullptr;
        }
    }

    pwmStart(pwm_driver, &pwm_cfg);

    /* Always start with pump disabled */
    disable();
}

void AP_SpraySystem_Pump::enable()
{
    set_pwm_output(current_throttle_value);
}

void AP_SpraySystem_Pump::disable()
{
    set_pwm_output(0);
}

void AP_SpraySystem_Pump::set_speed(uint16_t throttle_us)
{
    /* Sanity check throttle value, must be between 1050 - 1950 us */
    if (throttle_us < PUMP_MIN_THROTTLE_PERIOD || throttle_us > PUMP_MAX_THROTTLE_PERIOD)
    {
        return;
    }

    current_throttle_value = throttle_us;
    set_pwm_output(current_throttle_value);
}

inline void AP_SpraySystem_Pump::set_pwm_output(uint16_t throttle)
{
    pwmEnableChannel(pwm_driver, pump_pwm_channel, throttle);
}