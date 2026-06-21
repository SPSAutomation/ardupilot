#pragma once

#include <AP_SpraySystem/AP_SpraySystem_FlowSensor.hpp>
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
    AP_SpraySystem() = default;
    ~AP_SpraySystem() = default;

    /* Do not allow copies
     * This is a singleton against my will */
    CLASS_NO_COPY(AP_SpraySystem);

    static AP_SpraySystem *get_singleton();
    static AP_SpraySystem *_singleton;

    void init();

    void update();
    SprayScheduleResult enqueue_spray_routine(SprayRoutine routine);

    void set_spray_nozzle_open(bool open);
    void set_return_line_open(bool open);

    void set_constant_flow_rate(uint32_t flow_rate_ml_min);
    void stop_flow();

    void set_pump_speed(uint32_t pump_throttle_value);

    uint32_t get_current_pressure_psi();

    uint32_t get_current_flow_rate_ml_min();

    static const struct AP_Param::GroupInfo     var_info[];

    void transmit_status();

private:
    void flow_pid_step();

    AC_PID * pid_instance;

    AP_SpraySystem_FlowSensor * flow_sensor;
};

namespace AP {
    AP_SpraySystem &spray_system();
};


#endif

