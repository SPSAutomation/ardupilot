/// @file   AC_SpotSprayer.h
/// @brief  Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        - set RC7_OPTION or RC8_OPTION parameter to 15 to allow turning the sprayer on/off from one of these channels
        - set SERVO10_FUNCTION to 22 to enable the servo output controlling the pump speed on servo-out 10
        - set SERVO11_FUNCTION to 23 to enable the servo output controlling the spinner on servo-out 11
        - ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        - set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        - set the SPRAY_PUMP_RATE to the value the pump servo should move to when the vehicle is travelling at 1m/s. This is expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        - set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        - set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/
#pragma once

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <canard.h>
#include <dronecan_msgs.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Logger/AP_Logger_config.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define AC_SPRAYER_DEFAULT_FLOW_RATE_LOW    500
#define AC_SPRAYER_DEFAULT_FLOW_RATE_MID    1000
#define AC_SPRAYER_DEFAULT_FLOW_RATE_HIGH   1500
#define AC_SPRAYER_DEFAULT_PRESSURE         900

#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     ///< delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump

#define MSG_TIMEOUT                         3000

#define ERROR_MSG_TIMEOUT                   10000

#ifndef HAL_SPOT_SPRAYER_ENABLED
#define HAL_SPOT_SPRAYER_ENABLED 1
#endif

#if HAL_SPOT_SPRAYER_ENABLED

/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class AC_SpotSprayer {
public:
    AC_SpotSprayer();

    enum class FlowRate {
        LOW = 0,
        MIDDLE = 1,
        HIGH = 2,
    };

    /* Do not allow copies */
    CLASS_NO_COPY(AC_SpotSprayer);

    static AC_SpotSprayer *get_singleton();
    static AC_SpotSprayer *_singleton;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_SprayInfo &msg);
    static AC_SpotSprayer* get_dronecan_backend(AP_DroneCAN* ap_dronecan);

    /// run - allow or disallow spraying to occur
    void run(bool true_false);

    void set_flow_rate(FlowRate flow_rate);
    uint16_t get_flow_rate();
    uint16_t get_pressure();

    /// spraying - returns true if spraying is actually happening
    bool spraying() const { return _spraying; }
    bool enabled() const { return (bool) _enabled; }

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len, float useful_load) const;

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Int8         _enabled;               ///< top level enable/disable control
    AP_Int16        _flow_rate_low;         ///< Desired low flow rate
    AP_Int16        _flow_rate_mid;         ///< Desired middle flow rate
    AP_Int16        _flow_rate_high;        ///< Desired high flow rate
    AP_Int16        _pressure;              ///< Desired pump pressure

private:

    bool _spraying;
    FlowRate _current_flow_rate;

    // internal variables
    uint32_t        _speed_over_min_time;   ///< time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  ///< time at which we fell below speed minimum

    uint32_t    last_reading_ms;
    uint16_t    measured_flow_rate;
    uint16_t    measured_pressure;
    float       spray_level;
    uint8_t     error_flags;

    float       _reported_weight;

    HAL_Semaphore _sem;

    AP_DroneCAN* _ap_dronecan;

    uint32_t _last_fault_msg_ms;

#if HAL_LOGGING_ENABLED
    // method and state to write and entry to the onboard log:
    void log_write();
    uint32_t last_logged_reading_ms;
#endif

};

namespace AP {
    AC_SpotSprayer *spot_sprayer();
};
#endif // HAL_SPRAYER_ENABLED
