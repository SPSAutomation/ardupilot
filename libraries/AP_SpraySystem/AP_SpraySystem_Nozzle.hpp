#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

#define NOZZLE_UPDATE_RATE_HZ 1000
#define NOZZLE_PWM_FREQUENCY_HZ 50

/**
 * @brief This class provides a driver for externally connected spray nozzle solenoids.
 * Solenoids are controlled using a manual PWM. This is primarily to prevent overheating
 * in the return line, as the main spray nozzle solenoid is either entirely on or entirely off
 */
class AP_SpraySystem_Nozzle
{
public:
    AP_SpraySystem_Nozzle() = default;

    /**
     * @brief Initialises the nozzle control driver, assigns an output and calculates
     * the number of on and off ticks based on the active duty cycle.
     *
     * @param nozzle_ctrl_output GPIO instance for controlling the connected nozzle
     * @param duty duty cycle at which to run the PWM
     */
    void init(AP_HAL::HAL& hal, uint8_t ctrl_pin, uint32_t duty_percent);

    /**
     * @brief Iterates nozzle on/off timer counts and toggles the solenoid state if necessary.
     */
    void update();

    /**
     * @brief Sets the nozzle to be open using the configured frequency and duty cycle
     */
    void open();

    /**
     * @brief Closes the connected nozzle
     */
    void close();

    /**
     * @brief Gets the current state of the nozzle solenoid
     *
     * @return true if the nozzle is open (i.e. the PWM is running), false otherwise
     */
    bool is_open();

private:

    /**
     * @brief Directly sets the state of the solenoid (open or closed)
     *
     * @param open true to open the solenoid, false to close
     */
    void set_solenoid_open(bool open);

    /**
     * Current state of the nozzle
     */
    bool nozzle_open{false};

    /**
     * Current state of the solenoid
     */
    bool solenoid_open{false};

    /**
     * Current open/close tick count
     */
    uint32_t open_count{0};
    uint32_t close_count{0};

    /**
     * Target open/close counts. Nozzle state will toggle
     * when these targets are reached
     */
    uint32_t open_count_target{0};
    uint32_t close_count_target{0};

    /**
     * HAL reference for output pin control
     */
    AP_HAL::HAL& hal_ref;
    uint8_t nozzle_ctrl_pin;
};

#endif