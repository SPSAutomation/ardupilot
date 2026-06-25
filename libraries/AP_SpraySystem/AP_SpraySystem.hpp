#pragma once

#include <AP_SpraySystem/AP_SpraySystem_FlowSensor.hpp>
#include <AP_SpraySystem/AP_SpraySystem_Nozzle.hpp>
#include <AP_SpraySystem/AP_SpraySystem_Pump.hpp>
#include <AP_SpraySystem/AP_SpraySystem_PressureSensor.hpp>
#include <AC_PID/AC_PID.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <dronecan_msgs.h>

#if AP_PERIPH_BFD_SPRAY_SYSTEM_ENABLED

#define SPRAY_ROUTINE_MAX_QUEUE_LENGTH      3
#define FLOW_CONTROLLER_UPDATE_PERIOD_MS    10
#define AMOUNT_THRESHOLD_PROPORTION         0.1    // 10% threshold

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
     * @brief sets the current pump throttle value
     *
     * @param pump_throttle_value pump PWM throttle pulse width
     * between 1040 - 1950 us
     */
    void set_pump_speed(uint32_t pump_throttle_value);

    /**
     * @brief returns the most recent pressure reading from the sensor
     *
     * @return pressure value in mbar
     */
    uint32_t get_current_pressure_mbar();

    /**
     * @brief returns the most recent temperature reading from the sensor
     *
     * @return current temperature value in degrees c
     */
    float get_current_temperature_c();

    /**
     * @brief reads out the current flow sensor filtered rate
     *
     * @return current flow rate in ml / minute
     */
    uint32_t get_current_flow_rate_ml_min();

    /**
     * @brief handler for incoming global time sync broadcasts.
     * Updates the offset applied to the monotonic timestamp to
     * get a time matching that of the controller.
     *
     * @param timestamp timestamp provided by time sync message
     */
    void handle_time_sync_message(uint32_t timestamp);

    static const struct AP_Param::GroupInfo     var_info[];

private:

    static AP_SpraySystem *_singleton;

    /**
     * @brief increments the flow rate PID controller
     *
     * @param dt_ms time since last PID step in ms
     */
    void flow_pid_step(uint32_t dt_ms);

    /**
     * @brief Pulls the next spray routine from the queue
     * and prepares to spray at designated time
     */
    SprayScheduleResult schedule_next_spray_routine();

    /**
     * @brief checks whether it is time to start the next
     * scheduled routine
     */
    bool time_to_start_routine();

    /**
     * @brief starts the next spray routine
     */
    void start_routine();

    void end_routine();

    uint64_t get_current_time_millis();

    AC_PID * pid_instance;

    AP_SpraySystem_FlowSensor * flow_sensor;

    AP_SpraySystem_Nozzle * spray_nozzle;
    AP_SpraySystem_Nozzle * return_line;

    AP_SpraySystem_Pump * pump;

    AP_SpraySystem_PressureSensor * pressure_sensor;

    AP_Float _flow_sense_pulse_ul;

    /* Currently executed spray routine */
    SprayRoutine current_spray_routine;

    /* Amount of time the current routine has been actively spraying */
    uint32_t time_spraying_ms;

    /* Variables used for time synchronisation with controller */
    int64_t montonic_clock_offset{0};
    uint64_t last_sync_rx_timestamp{0};

    /* Current state of the spray system */
    SpraySchedulerState current_state{SpraySchedulerState::IDLE};

    /* Callback for when routines are complete */
    void (*routine_complete_cb)(float, uint32_t, bool){nullptr};

    /* Current pump speed used by PID */
    uint16_t current_pump_speed_us;

    /* Queue for scheduled spray routines */
    ObjectBuffer<SprayRoutine> spray_routine_queue{SPRAY_ROUTINE_MAX_QUEUE_LENGTH};
};

namespace AP {
    AP_SpraySystem *spray_system();
};


#endif

