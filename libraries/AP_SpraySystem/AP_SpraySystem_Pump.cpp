#include "AP_SpraySystem_Pump.hpp"

AP_SpraySystem_Pump::AP_SpraySystem_Pump(PWMDriver *driver, uint8_t pwm_channel)
{
    pwm_driver = driver;
    pump_pwm_channel = pwm_channel;

    static const PWMConfig cfg = {
            PWM_TIMER_CLOCK_HZ,
            PWM_TIMER_PERIOD,
            nullptr,
            {
                    {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // TIM3_CH1 / PB4
                    {PWM_OUTPUT_DISABLED, nullptr},
                    {PWM_OUTPUT_DISABLED, nullptr},
                    {PWM_OUTPUT_DISABLED, nullptr},
            },
            0,
            0
    };

    pwmStart(pwm_driver, &cfg);

    /* Always start with pump disabled */
    disable();
}

void AP_SpraySystem_Pump::enable()
{
    set_pwm_output(pump_pwm_channel, current_throttle_value);
}

void AP_SpraySystem_Pump::disable()
{
    set_pwm_output(pump_pwm_channel, 0);
}

void AP_SpraySystem_Pump::set_speed(uint16_t throttle_us)
{
    /* Sanity check throttle value, must be between 1050 - 1950 us */
    if (throttle_us < PUMP_MIN_THROTTLE_PERIOD || throttle_us > PUMP_MAX_THROTTLE_PERIOD)
    {
        return;
    }

    current_throttle_value = throttle_us;
    set_pwm_output(pump_pwm_channel, current_throttle_value);
}

inline void AP_SpraySystem_Pump::set_pwm_output(uint16_t throttle)
{
    pwmEnableChannel(pwm_driver, pump_pwm_channel, throttle);
}