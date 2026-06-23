#include "AP_SpraySystem_Nozzle.hpp"

const AP_HAL::HAL& hal_ref = AP_HAL::get_HAL();

void AP_SpraySystem_Nozzle::init(uint32_t ctrl_pin, uint32_t duty_percent)
{
    nozzle_ctrl_pin = ctrl_pin;

    /* Configure pin as an output */
    hal_ref.gpio->pinMode(nozzle_ctrl_pin, HAL_GPIO_OUTPUT);
    set_solenoid_open(false);

    float pwm_period_ms = (1.0f / NOZZLE_PWM_FREQUENCY_HZ) * 1000;

    float total_pwm_period_ticks = pwm_period_ms / NOZZLE_UPDATE_PERIOD_MS;

    /*
     * Calculate the number of ticks for each state of the nozzle to open/close
     */
    open_count_target = static_cast<uint32_t> ((duty_percent / 100.0f) * total_pwm_period_ticks);
    close_count_target = static_cast<uint32_t>(total_pwm_period_ticks) - open_count_target;
}

void AP_SpraySystem_Nozzle::update()
{
    /* If the solenoid is open, run the PWM */
    if (nozzle_open)
    {
        /* PWM control is performed here */
        if (hal_ref.gpio->read(nozzle_ctrl_pin))
        {
            /* Increment open count */
            open_count++;

            /* Only close the solenoid if the duty cycle is less than 100% */
            if (open_count >= open_count_target && close_count_target > 0)
            {
                set_solenoid_open(false);
                close_count = 0;
            }
        }
        else
        {
            /* Increment close count */
            close_count++;

            /* Only open the solenoid if the duty cycle is greater than 0 */
            if (close_count >= close_count_target && open_count_target > 0)
            {
                set_solenoid_open(true);
                open_count = 0;
            }

        }
    }
    /* If the nozzle is supposed to be closed, close the solenoid */
    else
    {
        /* Nozzle is closed, ensure solenoid is shut */
        if (hal_ref.gpio->read(nozzle_ctrl_pin))
        {
            set_solenoid_open(false);
        }
    }
}

void AP_SpraySystem_Nozzle::open()
{
    /* Reset all counts */
    open_count = 0;
    close_count = 0;

    /* Emable the PWM */
    nozzle_open = true;

    /* Actuate nozzle immediately */
    set_solenoid_open(true);
}

void AP_SpraySystem_Nozzle::close()
{
    /* Disable the PWM, update() function will close the solenoid */
    nozzle_open = false;
    /* Actuate solenoid immediately */
    set_solenoid_open(false);
}

bool AP_SpraySystem_Nozzle::is_open()
{
    return nozzle_open;
}

inline void AP_SpraySystem_Nozzle::set_solenoid_open(bool open)
{
    hal_ref.gpio->write(nozzle_ctrl_pin, open ? 1 : 0);
}