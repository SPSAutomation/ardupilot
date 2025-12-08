// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Generator_Backend.h"

#if AP_GENERATOR_GX_16_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <canard.h>
#include <dronecan_msgs.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>


#define EXTENDER_PREARM_TEMP 35
#define EXTENDER_MAINTENANCE_SCHEDULE 720000 // 200 Hours in seconds

class AP_Generator_GX_16 : public AP_Generator_Backend
{

public:
    using AP_Generator_Backend::AP_Generator_Backend;

    AP_BattMonitor::Failsafe update_failsafes() const override;

    // init should be called at vehicle startup to get the generator library ready
    void init(void) override;
    // update should be called regularly to update the generator state
    void update(void) override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_GX16ExtenderInfo &msg);
    static void handle_fans(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_FanStatus &msg);
    static AP_Generator_GX_16* get_dronecan_backend(AP_DroneCAN* ap_dronecan);

    // methods to control the generator state:
    bool stop(void) override;
    bool idle(void) override;
    bool run(void) override;

    void shutdown_on_land(bool shutdown) override {shutdown_on_landing = shutdown; };

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
    enum class WorkingState {           // HEX      DECIMAL
        STOP = 0x00,                    // 0x00     0
        MOTORCONTROL = 0x01,            // 0x01     1
        MOTORRESPONSE = 0x02,           // 0x02     2
        CONTROLCMD_RUNCAN = 0x10,       // 0x10     16
        CONTROLCMD_RUNCOM = 0x20,       // 0x20     32
        CONTROLCMD_RUNREMOTE = 0x30,    // 0x30     48
        CRANK = 0x40,                   // 0x40     64
        IDLE = 0x80,                    // 0x80     128
        RUN = 0xc0                      // 0xc0     192
    };

    // methods and state to record pilot desired runstate and actual runstate:
    enum class RunState {
        STOP = 17,
        IDLE = 18,
        RUN = 19,
    };

private:

    enum class OverlimitFault1 : uint8_t {      // HEX      DECIMAL     EXPLANATION
        OVER_VOLTAGE_ERROR      = (1U << 0),    // 0x00001  1           Output voltage exceeds the limit
        OVER_CURRENT_ERROR      = (1U << 1),    // 0x00002  2           Output current exceeds limit
        BATT_CHRG_OVER_LIMIT    = (1U << 2),    // 0x00004  4           Battery charging current exceeds the limit
        COOL1_OVER_TEMP         = (1U << 3),    // 0x00008  8           Refrigerant temperature 1 exceeds limit
        COOL2_OVER_TEMP         = (1U << 4),    // 0x00010  16          Coolant temperature 2 exceeds limit
        COIL_OVER_TEMP          = (1U << 5),    // 0x00020  32          Motor coil temperature exceeds limit
        INTAKE_OVER_TEMP        = (1U << 6),    // 0x00040  64          Intake temperature exceeds limit
        RAIL_OVER_TEMP          = (1U << 7)     // 0x00080  128         12V/7.4V/5.1V Overlimit
    };

    enum class OverlimitFault2 : uint8_t {      // HEX      DECIMAL     EXPLANATION
        ENG_OVER_SPEED          = (1U << 0),    // 0x00001  1           Engine speed limit exceeded
        IGNITION_COIL_ERROR     = (1U << 1),    // 0x00002  2           Ignition coil failure (overcurrent)
        FUEL_INJECTOR_ERROR     = (1U << 2),    // 0x00004  4           Fuel injector failure (overcurrent)
        // RESERVED             = (1U << 3),    // 0x00008  8           reserved
        OIL_PUMP_ERROR          = (1U << 4),    // 0x00010  16          Oil pump failure (overcurrent)
        // RESERVED             = (1U << 5),    // 0x00020  32          reserved
        // RESERVED             = (1U << 6),    // 0x00040  64          reserved
        // RESERVED             = (1U << 7),    // 0x00080  128         reserved
    };

    enum class GeneralFault0 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        OUTPUT_CURRENT_SENSOR_ERROR     = (1U << 0),    // 0x00001  1           Output current sensor unavailable (powershort circuit)
        BATT_CURRENT_SENSOR_ERROR       = (1U << 1),    // 0x00002  2           Battery current sensor is unavailable (powershort circuit)
        COIL_SENSOR_ERROR               = (1U << 2),    // 0x00004  4           Motor coil temperature sensor unavailable (ground short circuit)
        THROTTLE_SENSOR_ERROR           = (1U << 3),    // 0x00008  8           Throttle position sensor is unavailable (power short circuit)
        COOL1_SENSOR_ERROR              = (1U << 4),    // 0x00010  16          The coolant temperature 1 sensor is not available (ground short circuit)
        COOL2_SENSOR_ERROR              = (1U << 5),    // 0x00020  32          The coolant temperature 2 sensor is not available (ground short circuit)
        BARO_SENSOR_ERROR               = (1U << 6),    // 0x00040  64          Atmospheric pressure sensor is not available (power short circuit)
        INTAKE_TEMP_SENSOR_ERROR        = (1U << 7),    // 0x00080  128         Intake temperature sensor unavailable (ground short circuit)
    };

    enum class GeneralFault1 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        OIL_LEVEL_SENSOR_ERROR          = (1U << 0),    // 0x00001  1           Oil level sensor is unavailable (power short circuit)
        IGNITION_COIL_NOT_INSTALLED     = (1U << 1),    // 0x00002  2           Ignition coil 1 not installed (ground short circuit)
        // RESERVED                     = (1U << 2),    // 0x00004  4           reserved
        INJECTOR_NOT_INSTALLED          = (1U << 3),    // 0x00008  8           Injector 1 not installed (ground short circuit) (keep)
        // RESERVED                     = (1U << 4),    // 0x00010  16          reserved
        BUS_DETECTION_FAILED            = (1U << 5),    // 0x00020  32          The bus voltage detection circuit failed to detect bus voltage (not installed)
        BAD_RC_ERROR                    = (1U << 6),    // 0x00040  64          Invalid remote control signal duty cycle (out of range)
        // RESERVED                     = (1U << 7),    // 0x00080  128         reserved     
    };

    enum class GeneralFault2 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        OUTPUT_CURRENT_SENSOR_MISSING   = (1U << 0),    // 0x00001  1           Output current sensor not installed (ground short circuit)
        BATT_CURRENT_SENSOR_MISSING     = (1U << 1),    // 0x00002  2           Battery current sensor not installed (ground short circuit) 
        COIL_SENSOR_MISSING             = (1U << 2),    // 0x00004  4           Motor coil temperature sensor not installed (ground open circuit)
        THROTTLE_SENSOR_MISSING         = (1U << 3),    // 0x00008  8           Throttle position sensor not installed (ground short circuit)
        COOL1_SENSOR_MISSING            = (1U << 4),    // 0x00010  16          1 coolant temperature sensor not installed (ground open circuit)
        COOL2_SENSOR_MISSING            = (1U << 5),    // 0x00020  32          2 coolant temperature sensors not installed (ground open circuit)
        BARO_SENSOR_MISSING             = (1U << 6),    // 0x00040  64          Atmospheric pressure sensor not installed (ground short circuit)
        INTAKE_TEMP_SENSOR_MISSING      = (1U << 7),    // 0x00080  128         The intake temperature sensor is not installed (ground open circuit)
    };

    enum class GeneralFault3 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        // RESERVED                     = (1U << 0),    // 0x00001  1           reserved
        OIL_PUMP_MISSING                = (1U << 1),    // 0x00002  2           Oil pump not installed (ground short circuit)
        // RESERVED                     = (1U << 2),    // 0x00004  4           reserved
        // RESERVED                     = (1U << 3),    // 0x00008  8           reserved
        // RESERVED                     = (1U << 4),    // 0x00010  16          reserved
        OIL_SENSOR_ERROR                = (1U << 5),    // 0x00020  32          Oil level sensor not installed (ground short circuit)
        // RESERVED                     = (1U << 6),    // 0x00040  64          reserved
        // RESERVED                     = (1U << 7),    // 0x00080  128         reserved
    };

    enum class AbnormalAlert0 : uint8_t {               // HEX      DECIMAL     EXPLANATION
        SHUTDOWN_OVER_VOLTAGE           = (1U << 0),    // 0x00001  1           Shutdown protection due to excessively high output voltage
        OUTPUT_OVER_CURRENT_ALARM       = (1U << 1),    // 0x00002  2           Output current is too high
        BATT_OVER_CURRENT_ALARM         = (1U << 2),    // 0x00004  4           Battery charging current is too high
        OVER_RPM_ALARM                  = (1U << 3),    // 0x00008  8           Engine RPM is too high
        COOL1_OVER_TEMP_ALARM           = (1U << 4),    // 0x00010  16          The coolant 1 temperature is too high
        COOL2_OVER_TEMP_ALARM           = (1U << 5),    // 0x00020  32          The coolant 2 temperature is too high
        COIL_OVER_TEMP_ALARM            = (1U << 6),    // 0x00040  64          The motor coil is overheating
        OVER_VOLTAGE_ALARM              = (1U << 7),    // 0x00080  128         Output voltage alarm
    };
    
    enum class AbnormalAlert1 : uint8_t {               // HEX      DECIMAL     EXPLANATION
        BARO_OVER_PRESSURE              = (1U << 0),    // 0x00001  1           The atmospheric pressure is too high
        BARO_UNDER_PRESSURE             = (1U << 1),    // 0x00002  2           Too low atmospheric pressure
        INTAKE_OVER_TEMP                = (1U << 2),    // 0x00004  4           The intake temperature is too high
        LOW_OIL_ALARM                   = (1U << 3),    // 0x00008  8           Too low oil level
        VREF_VOLTAGE_ALARM              = (1U << 4),    // 0x00010  16          VDDA (VREF) voltage is too high or too low
        VBATT_VOLTAGE_ALARM             = (1U << 5),    // 0x00020  32          VBAT voltage too high or too low
        RTC_ALARM                       = (1U << 6),    // 0x00040  64          RTC not synchronized (since this startup)
        SHUTDOWN_LOW_VOLTAGE            = (1U << 7),    // 0x00080  128         Shutdown protection due to low output voltage
    };

    enum class AbnormalAlert2 : uint8_t {               // HEX      DECIMAL     EXPLANATION
        LOW_OUTPUT_VOLTAGE_ALARM        = (1U << 0),    // 0x00001  1           Output voltage low alarm
        // RESEVERED                    = (1U << 1),    // 0x00002  2           reserved
        THROTTLE_LEARNING_ALARM         = (1U << 2),    // 0x00004  4           Steering gear not completed self-learning
        THROTTLE_ABNORMAL_ALARM         = (1U << 3),    // 0x00008  8           Throttle opening abnormal
        // RESEVERED                    = (1U << 4),    // 0x00010  16          reserved
        // RESEVERED                    = (1U << 5),    // 0x00020  32          reserved
        // RESEVERED                    = (1U << 6),    // 0x00040  64          reserved
        // RESEVERED                    = (1U << 7),    // 0x00080  128         reserved
    };

    enum class AbnormalAlert3 : uint8_t {               // HEX      DECIMAL     EXPLANATION
        RAIL_12V_OVER_VOLTAGE           = (1U << 0),    // 0x00001  1           12V voltage is too high
        RAIL_12V_UNDER_VOLTAGE          = (1U << 1),    // 0x00002  2           12V voltage is too low
        RAIL_7V4_OVER_VOLTAGE           = (1U << 2),    // 0x00004  4           7.4V voltage is too high
        RAIL_7V4_UNDER_VOLTAGE          = (1U << 3),    // 0x00008  8           7.4V voltage is too low
        RAIL_5V1_OVER_VOLTAGE           = (1U << 4),    // 0x00010  16          Sensor 5.1V voltage is too high
        RAIL_5V1_UNDER_VOLTAGE          = (1U << 5),    // 0x00020  32          Sensor 5.1V voltage is too low
        MAINTENANCE_ALARM               = (1U << 6),    // 0x00040  64          Maintenance time expired alert
        LOCKOUT_ALARM                   = (1U << 7),    // 0x00080  128         Lockout time expiration alert
    };

    enum class SystemStatus0 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        SYSTEM_ON                       = (1U << 0),    // 0x00001  1           Start/stop system: Off (0) / On (1)
        IGNITION_COIL_ENABLED           = (1U << 1),    // 0x00002  2           Ignition coil enable: Disabled (0) / Enabled (1)
        INJURER_ENABLED                 = (1U << 2),    // 0x00004  4           Injurer enabled: Disabled (0) / Enabled (1)
        OIL_PUMP_ENABLED                = (1U << 3),    // 0x00008  8           Oil pump enable: Disabled (0) / Enabled (1)
        PROTECTION_SHUTDOWN             = (1U << 4),    // 0x00010  16          Fault protection shutdown
        LOW_ENGINE_SPEED                = (1U << 5),    // 0x00020  32          Engine low speed
        WEAK_SYNCHRONIZATION            = (1U << 6),    // 0x00040  64          Weak synchronization:Not synced (0)/Synced (1)
        STRONG_SYNCHRONIZATION          = (1U << 7),    // 0x00080  128         Strong synchronization:Not synchronized (0)/Synchronized (1)
    };
    
    enum class SystemStatus1 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        FUEL_INJECTOR_STATUS            = (1U << 0),    // 0x00001  1           Fuel injection status: off (0)/on (1)
        IGNITION_COIL_CONTROL           = (1U << 1),    // 0x00002  2           Ignition coil control: off (0)/on (1)
        FUEL_INJECTOR_CONTROL           = (1U << 2),    // 0x00004  4           Fuel injector control: off (0)/on (1)
        OIL_PUMP_CONTROL                = (1U << 3),    // 0x00008  8           Oil pump control: off (0)/on (1)
        COOLANT_PUMP_CONTROL            = (1U << 4),    // 0x00010  16          Coolant pump control: off (0)/on (1)
        FAN_ON                          = (1U << 5),    // 0x00020  32          Fan: Off (0) / On (1)
        IGNITION_STATUS                 = (1U << 6),    // 0x00040  64          Ignition status: off (0)/on (1)
        EXTERNAL_START                  = (1U << 7),    // 0x00080  128         External start/stop command: Stop (0) / Start (1)
    };

    enum class SystemStatus2 : uint8_t {                // HEX      DECIMAL     EXPLANATION
        // RESERVED                     = (1U << 0),    // 0x00001  1           reserved
        // RESERVED                     = (1U << 1),    // 0x00002  2           reserved
        ABNORMAL_ALARM                  = (1U << 2),    // 0x00004  4           Abnormal alarm indicator: off (0)/on (1)
        STATUS_INDICATOR                = (1U << 3),    // 0x00008  8           Status indicator: off (0)/on (1)
        // RESERVED                     = (1U << 4),    // 0x00001  1           reserved
        // RESERVED                     = (1U << 5),    // 0x00002  2           reserved
        // RESERVED                     = (1U << 6),    // 0x00001  1           reserved
        // RESERVED                     = (1U << 7),    // 0x00002  2           reserved
    };


    RunState pilot_desired_runstate = RunState::STOP;
    RunState commanded_runstate = RunState::STOP;  // output is based on this
    void set_pilot_desired_runstate(RunState newstate) {
        pilot_desired_runstate = newstate;
    }
    void update_runstate();

    // CAN message fields of the generator
    uint8_t         working_state;
    float           coolant_temp_1;                 // degree Celsius
    float           coolant_temp_2;                 // degree Celsius
    float           coil_temp;                      // degree Celsius
    uint16_t        engine_speed;                   // RPM
    float           output_voltage;                    // V
    float           output_current;                 // A
    float           battery_current;                // A
    float           target_throttle_position;       // %
    float           actual_throttle_position;       // %
    uint8_t         baro;
    float           IAT;                            // degree Intake Air Temperature Maybe?
    uint16_t        fuel_consumption;                
    uint8_t         fuel_level;                     // %
    float           rail_12V;                       // V
    float           rail_5V1;                       // V
    float           rail_7V4;                       // V
    float           rail_VBATT;                     // V
    float           rail_VREF;                      // V
    uint8_t         EmgST0;                         // Overlimit fault 0  $1EC
    uint8_t         EmgST1;                         // Overlimit fault 1
    uint8_t         ErrST0;                         // General fault status 0
    uint8_t         ErrST1;                         // General fault status 1
    uint8_t         ErrST2;                         // General fault status 2
    uint8_t         ErrST3;                         // General fault status 3
    uint8_t         AlmST0;                         // abnormal alarm 0  $1ED
    uint8_t         AlmST1;                         // abnormal alarm 1
    uint8_t         AlmST2;                         // abnormal alarm 2
    uint8_t         AlmST3;                         // abnormal alarm 3
    uint8_t         SysST0;                         // System status 0
    uint8_t         SysST1;                         // System status 1
    uint8_t         SysST2;                         // System status 2
    uint8_t         RxCounter;                      // CAN receive error counter  $1EF
    uint8_t         TxCounter;                      // CAN sending error counter
    uint8_t         BusError;                       // CAN bus error
    uint8_t         TxAccTime;                      // Accumulated time of unsuccessful message transmission
    uint8_t         RxCtrlAccTime;                  // Accumulated time of not receiving control messages
    uint8_t         RxCtrlAccCount;                 // Accumulated number of errors in receiving control messages
    uint8_t         RxMotorAccTime;                 // Accumulated time of not receiving motor message
    uint8_t         RxMotorAccCount;                // Accumulated number of errors in receiving motor messages

    struct fanStatus {
        uint8_t id;
        int32_t rpm;
        uint8_t power_pct;
        uint8_t health;
    };

    std::vector<fanStatus> fanInfo;

    bool shutdown_on_landing;


#if HAL_LOGGING_ENABLED
    // method and state to write and entry to the onboard log:
    void Log_Write();
    uint32_t last_logged_reading_ms;
#endif

    uint32_t last_reading_ms;

    const char* error_strings[12] = {
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

    bool is_critical_error(const uint32_t err_in) const;
    bool is_low_error(const uint32_t err_in) const;

    // if we are emitting warnings about the generator requiring
    // maintenance, this is the last time we sent the warning:
    uint32_t last_maintenance_warning_ms;
    mutable uint32_t last_error_sent;
    uint32_t last_fan_warning_ms;
};
#endif
