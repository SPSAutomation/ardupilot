#pragma once

#include "AP_Generator_config.h"

#if HAL_GENERATOR_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

class AP_Generator_Backend;
class AP_Generator_IE_650_800;
class AP_Generator_IE_2400;
class AP_Generator_RichenPower;
class AP_Generator_GX_7;
class AP_Generator_GX_16;

class AP_Generator
{
    friend class AP_Generator_Backend;
    friend class AP_Generator_IE_650_800;
    friend class AP_Generator_IE_2400;
    friend class AP_Generator_RichenPower;
    friend class AP_Generator_GX_7;
    friend class AP_Generator_GX_16;

public:
    // Constructor
    AP_Generator();

    // Do not allow copies
    CLASS_NO_COPY(AP_Generator);

    static AP_Generator* get_singleton();

    void init(void);
    void update(void);

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    AP_BattMonitor::Failsafe update_failsafes(void);

    // Helpers to retrieve measurements
    float get_voltage(void) const { return _voltage; }
    float get_current(void) const { return _current; }
    // get_fuel_remaining returns fuel remaining as a scale 0-1
    float get_fuel_remaining(void) const { return _fuel_remaining; }
    float get_batt_consumed(void) const { return _consumed_mah; }
    uint16_t get_rpm(void) const { return _rpm; }

    // Helpers to see if backend has a measurement
    bool has_current() const { return _has_current; }
    bool has_consumed_energy() const { return _has_consumed_energy; }
    bool has_fuel_remaining() const { return _has_fuel_remaining; }

    // healthy() returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating any errors.
    bool healthy(void) const { return _healthy; }

    // Generator controls must return true if present in generator type
    bool stop(void);
    bool idle(void);
    bool run(void);

    void shutdown_on_land(bool shutdown);

    void send_generator_status(const class GCS_MAVLINK &channel);

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // bits which can be set in _options to modify generator behaviour:
    enum class Option {
        INHIBIT_MAINTENANCE_WARNINGS = 0,
    };

    bool option_set(Option opt) const {
        return (_options & 1U<<uint32_t(opt)) != 0;
    }
    enum class Type {
        GEN_DISABLED = 0,
#if AP_GENERATOR_IE_650_800_ENABLED
        IE_650_800 = 1,
#endif
#if AP_GENERATOR_IE_2400_ENABLED
        IE_2400 = 2,
#endif
#if AP_GENERATOR_RICHENPOWER_ENABLED
        RICHENPOWER = 3,
#endif
#if AP_GENERATOR_GX_7_ENABLED
        GX_7 = 4,
#endif
#if AP_GENERATOR_GX_16_ENABLED
        GX_16 = 5,
#endif
    };


    Type get_type(void) {return (Type)(uint8_t)_type;}

    uint8_t get_state(void) {return _state;}
    uint32_t get_state_change_time(void) {return _state_change_time;}
    uint8_t get_commanded_state(void) {return _commanded_state; }

    uint8_t get_options(void) {return _options;}

    int8_t get_low_fuel_failsafe_action(void) {return (int8_t) _fuel_low_fs;}
    int8_t get_crit_fuel_failsafe_action(void) {return (int8_t) _fuel_crit_fs;}
    int8_t get_off_failsafe_action(void) {return (int8_t) _off_fs;}
    int8_t get_low_fuel_failsafe_level(void) {return (int8_t) _fuel_low_percent;}
    int8_t get_crit_fuel_failsafe_level(void) {return (int8_t) _fuel_crit_percent;}
    int8_t get_error_failsafe_action(void) {return (int8_t) _error_fs;}
    int16_t get_last_service_time(void) {return (int16_t) _last_service_time;}
    int8_t get_prearm_fuel_level(void) {return (int8_t) _fuel_prearm_percent;}
    float get_fuel_tank_size(void) {return (float) _fuel_tank_size;}

private:

    // Pointer to chosen driver
    AP_Generator_Backend* _driver_ptr;

    // Parameters
    AP_Int8 _type; // Select which generator to use
    AP_Int32 _options; // Select which generator to use
    AP_Int8 _fuel_low_percent;
    AP_Int8 _fuel_low_fs; // Low fuel failsafe
    AP_Int8 _fuel_crit_percent;
    AP_Int8 _fuel_crit_fs; // Critical fuel failsafe
    AP_Int8 _off_fs;
    AP_Int8 _error_fs;
    AP_Int16 _last_service_time;
    AP_Int8 _fuel_prearm_percent;
    AP_Float _fuel_tank_size;
    

    // Helper to get param and cast to GenType
    Type type(void) const;

    // Front end variables
    float _voltage;
    float _current;
    float _fuel_remaining;  // 0-1
    bool _has_fuel_remaining;
    float _consumed_mah;
    uint16_t _rpm;
    bool _healthy;
    bool _has_current;
    bool _has_consumed_energy;
    uint8_t _state;
    uint8_t _commanded_state;
    uint32_t _state_change_time;

    static AP_Generator *_singleton;

};

namespace AP {
    AP_Generator *generator();
};
#endif
