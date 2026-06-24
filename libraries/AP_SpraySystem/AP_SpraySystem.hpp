#pragma once

#include <AP_SpraySystem/AP_SpraySystem_FlowSensor.hpp>
#include <AP_SpraySystem/AP_SpraySystem_Nozzle.hpp>
#include <AC_PID/AC_PID.h>
#include <dronecan_msgs.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

typedef struct
{
    uint16_t desired_spray_ml;
    uint16_t desired_flow_rate_ml_min;
    uint16_t time_allowed_ms;
    uint16_t time_spraying_ms;
    uint64_t start_time_ms;
} SprayRoutine;

enum class SprayScheduleResult
{
    SUCCESS = 0,
    QUEUE_FULL,
    QUEUE_EMPTY,
    INVALID_JOB,
    HARDWARE_FAIL,
    NOT_READY
};

enum class SpraySchedulerState
{
    IDLE = 0,
    SCHEDULED,
    RUNNING
};

class AP_SpraySystem
{
public:
    AP_SpraySystem();
    ~AP_SpraySystem() = default;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_SpraySystem);

    /**
     * @brief Gets the singleton instance of this class
     */
    static AP_SpraySystem *get_singleton();

    /**
     * @brief initialises the spray system
     */
    void init();

    /**
     * @brief update function to be called regularly
     */
    void update();

    /**
     * @brief queues up a spray routine to be performed at some point
     * in the future
     *
     * @param routine SprayRoutine struct instance with routine to be
     * performed
     *
     * @return SprayScheduleResult value indicating whether the
     * scheduling was successful
     */
    SprayScheduleResult enqueue_spray_routine(SprayRoutine routine);

    /**
     * @brief opens or closes the spray nozzle solenoid
     *
     * @param open true to open nozzle, false to close
     */
    void set_spray_nozzle_open(bool open);

    /**
     * @brief opens or closes the return line solenoid
     *
     * @param open true to open solenoid, false to close
     */
    void set_return_line_open(bool open);

    /**
     * @brief sets the spray system to spray at a constant flow rate
     * indefinitely until stopped manually
     *
     * @param flow_rate_ml_min target flow rate in ml / minute
     */
    void set_constant_flow_rate(uint32_t flow_rate_ml_min);

    /**
     * @brief closes nozzle and return line and shuts down pump
     */
    void stop_flow();

    /**
     * @brief sets the current pump throttle value
     *
     * @param pump_throttle_value pump PWM throttle pulse width
     * between 1040 - 1950 us
     */
    void set_pump_speed(uint32_t pump_throttle_value);

    /**
     * @brief returns the most recent pressure reading from the sensor
     *
     * @return pressure value in psi
     */
    uint32_t get_current_pressure_psi();

    /**
     * @brief reads out the current flow sensor filtered rate
     *
     * @return current flow rate in ml / minute
     */
    uint32_t get_current_flow_rate_ml_min();

    static const struct AP_Param::GroupInfo     var_info[];

private:

    static AP_SpraySystem *_singleton;

    /**
     * @brief increments the flow rate PID controller
     */
    void flow_pid_step();

    uint32_t nozzle_last_update_ms{0};

    AC_PID * pid_instance;

    AP_SpraySystem_FlowSensor * flow_sensor;

    AP_SpraySystem_Nozzle spray_nozzle;

    AP_Float _flow_sense_pulse_ul;
};

namespace AP {
    AP_SpraySystem *spray_system();
};


#endif

