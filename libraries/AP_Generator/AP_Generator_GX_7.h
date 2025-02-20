// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Generator_Backend.h"

#if AP_GENERATOR_GX_7_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <canard.h>
#include <dronecan_msgs.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdio.h>


#define EXTENDER_PREARM_TEMP 55
#define EXTENDER_MAINTAINANCE_SCHEDULE 720000 // 200 Hours in seconds

class AP_Generator_GX_7 : public AP_Generator_Backend
{

public:
    using AP_Generator_Backend::AP_Generator_Backend;

    AP_BattMonitor::Failsafe update_failsafes() const override;

    // init should be called at vehicle startup to get the generator library ready
    void init(void) override;
    // update should be called regularly to update the generator state
    void update(void) override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_ExtenderInfo &msg);
    static AP_Generator_GX_7* get_dronecan_backend(AP_DroneCAN* ap_dronecan);

    // methods to control the generator state:
    bool stop(void) override;
    bool idle(void) override;
    bool run(void) override;

    // method to send a GENERATOR_STATUS mavlink message
    void send_generator_status(const GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming.  Note
    // that if the generator has never sent us a message then these
    // automatically pass!
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update front end with voltage, current, and rpm values
    void update_frontend_readings(void);

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors.
    bool healthy() const override;

    // reported mode from the generator:
    enum class WorkingState {
        STOP = 0,
        CRANK = 1,
        RUN = 2,
        LOCK = 3,
        INHIBIT = 4,
    };

private:

    // methods and state to record pilot desired runstate and actual runstate:
    enum class RunState {
        STOP = 17,
        IDLE = 18,
        RUN = 19,
    };
    RunState pilot_desired_runstate = RunState::STOP;
    RunState commanded_runstate = RunState::STOP;  // output is based on this
    void set_pilot_desired_runstate(RunState newstate) {
        pilot_desired_runstate = newstate;
    }
    void update_runstate();

    enum class ExtenderError : uint32_t {             // HEX      DECIMAL
        LOCK_TIME_EXPIRE_ERROR  = (1U <<  0),   // 0x00001  1
        MAINTENANCE_TIME_ERROR  = (1U <<  1),   // 0x00002  2
        LOW_OIL_ERROR           = (1U <<  2),   // 0x00004  4
        SYSTEM_ERROR            = (1U <<  3),   // 0x00008  8
        COMMUNICATION_ERROR     = (1U <<  4),   // 0x00010  16
        COIL_OVER_TEMP_ERROR    = (1U <<  6),   // 0x00040  64
        COOLANT_OVER_TEMP_ERROR = (1U <<  7),   // 0x00080  128
        THROTTLE_ERROR          = (1U <<  9),   // 0x00200  512
        OVER_SPEED_ERROR        = (1U <<  12),  // 0x01000  4096
        OVER_CURRENT_ERROR      = (1U <<  13),  // 0x02000  8192
        LOW_VOLTAGE_ERROR       = (1U <<  14),  // 0x04000  16384
        OVER_VOLTAGE_ERROR      = (1U <<  15),  // 0x08000  32768
        LAST                    = (1U <<  16),  // 0x10000  65536
    };    


    // Current stats of generator
    uint16_t        engine_speed;                   // RPM
    uint16_t        throttle_position;              // %
    uint8_t         fuel_level;                     // %
    uint8_t         motor_temperature;              // Degree Celsius
    uint8_t         engine_cyclinder_temperature;   // Degree Celsius
    uint16_t        output_voltage;                 // Volts
    uint16_t        output_current;                 // Amps
    uint16_t        total_run_time;                 // Minutes
    uint32_t        extender_error;                 // Bitmask of ExtenderErrors
    WorkingState    working_state; 

#if HAL_LOGGING_ENABLED
    // method and state to write and entry to the onboard log:
    void Log_Write();
    uint32_t last_logged_reading_ms;
#endif

    uint32_t last_reading_ms;

    const char *error_strings[12] = {
        "Lock Time Expired",
        "Maintenance Required",
        "Low Oil",
        "System Error",
        "Communication Error",
        "Coils Overs Temperature",
        "Coolant Overs Temperature",
        "Throttle Error",
        "Over Speed",
        "Over Current",
        "Low Voltage Output",
        "Over Voltage Output",
    };

    // returns true if the generator should be allowed to move into
    // the "run" (high-RPM) state:
    bool generator_ok_to_run() const;

    // boolean so we can announce we've stopped the generator due to a
    // crash just once:
    bool vehicle_was_crashed;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    AP_DroneCAN* _ap_dronecan;

    // data and methods to handle time-in-idle-state:
    uint32_t idle_state_start_ms;

    uint32_t time_in_idle_state_ms() const {
        if (idle_state_start_ms == 0) {
            return 0;
        }
        return AP_HAL::millis() - idle_state_start_ms;
    }

    // check if the generator requires maintenance and send a message if it does:
    void check_maintenance_required();

    bool is_critical_error(const uint32_t err_in) const;
    bool is_low_error(const uint32_t err_in) const;

    // if we are emitting warnings about the generator requiring
    // maintenamce, this is the last time we sent the warning:
    uint32_t last_maintenance_warning_ms;
};
#endif
