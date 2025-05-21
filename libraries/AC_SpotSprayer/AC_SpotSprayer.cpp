#include "AC_SpotSprayer.h"

#if HAL_SPOT_SPRAYER_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_SpotSprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_SpotSprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FLOW_RATE_LOW
    // @DisplayName: Flow rate low
    // @Description: Desired low flow rate
    // @Units: ml/s
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("RATE_LOW",   1, AC_SpotSprayer, _flow_rate_low, AC_SPRAYER_DEFAULT_FLOW_RATE_LOW),

    // @Param: FLOW_RATE_MID
    // @DisplayName: Flow rate middle
    // @Description: Desired middle flow rate
    // @Units: ml/s
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("RATE_MID",   2, AC_SpotSprayer, _flow_rate_mid, AC_SPRAYER_DEFAULT_FLOW_RATE_MID),

    // @Param: FLOW_RATE_HIGH
    // @DisplayName: Flow rate high
    // @Description: Desired high flow rate
    // @Units: ml/s
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("RATE_HIGH",   3, AC_SpotSprayer, _flow_rate_high, AC_SPRAYER_DEFAULT_FLOW_RATE_HIGH),

    // @Param: PRES
    // @DisplayName: Pump pressure
    // @Description: Pump Pressure target
    // @Units: kpa
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("PRES",   4, AC_SpotSprayer, _pressure, AC_SPRAYER_DEFAULT_PRESSURE),

    // @Param: MAX_LOAD
    // @DisplayName: Max Useful Load
    // @Description: Maximum useful load
    // @Units: kg
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("MAX_LOAD",   5, AC_SpotSprayer, _useful_load, AC_SPRAYER_DEFAULT_USEFUL_LOAD),

    // @Param: VOL_LOW
    // @DisplayName: Volume low
    // @Description: Desired low Volume
    // @Units: ml
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("VOL_LOW",   6, AC_SpotSprayer, _volume_low, AC_SPRAYER_DEFAULT_VOLUME_LOW),

    // @Param: VOL_MID
    // @DisplayName: Volume middle
    // @Description: Desired middle volume
    // @Units: ml
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("VOL_MID",   7, AC_SpotSprayer, _volume_mid, AC_SPRAYER_DEFAULT_VOLUME_MID),

    // @Param: VOL_HIGH
    // @DisplayName: Volume high
    // @Description: Desired high volume
    // @Units: ml
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("VOL_HIGH",   8, AC_SpotSprayer, _volume_high, AC_SPRAYER_DEFAULT_VOLUME_HIGH),

    // @Param: MODE
    // @DisplayName: Spot sprayer mode
    // @Description: Flowrate or volume mode
    // @Values: 0:Flowrate,1:Volume
    // @User: Standard
    AP_GROUPINFO("MODE",   9, AC_SpotSprayer, _mode, AC_SPRAYER_DEFAULT_MODE),

    // @Param: PULSE
    // @DisplayName: Pulse Volume
    // @Description: Sprayer Pulse Volume
    // @Units: ml
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("PULSE",   10, AC_SpotSprayer, _pulse, AC_SPRAYER_DEFAULT_PULSE),

    AP_GROUPEND
};

AC_SpotSprayer::AC_SpotSprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Get the AP_Sprayer singleton
 */
AC_SpotSprayer *AC_SpotSprayer::_singleton;
AC_SpotSprayer *AC_SpotSprayer::get_singleton()
{
    return _singleton;
}


//links the rangefinder uavcan message to this backend
void AC_SpotSprayer::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    get_dronecan_backend(ap_dronecan);

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("measurement_sub");
    }
}

//Method to find the backend relating to the node id
AC_SpotSprayer* AC_SpotSprayer::get_dronecan_backend(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    AC_SpotSprayer* driver = AP::spot_sprayer();

    //Double check if the driver was initialised as UAVCAN Type
    if (driver != nullptr) {
        if (driver->_ap_dronecan == ap_dronecan) {
            return driver;
        } else {
            driver->_ap_dronecan = ap_dronecan;
            return driver;
        }
    }

    return driver;
}

//RangeFinder message handler
void AC_SpotSprayer::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_SprayInfo &msg)
{
    AC_SpotSprayer* driver = get_dronecan_backend(ap_dronecan);
    if (driver == nullptr)
    {
        return;
    }

    WITH_SEMAPHORE(driver->_sem);
    driver->last_reading_ms = AP_HAL::millis();
    driver->measured_flow_rate = msg.flow_rate;
    driver->measured_pressure = msg.pressure;
    driver->spray_level = msg.spray_remaining;
    driver->total_sprayed_volume = msg.total_sprayed_vol;
    driver->armed_sprayed_volume = msg.armed_sprayed_vol;
    driver->last_tree_volume = msg.last_tree_vol;
    driver->error_flags = msg.error_flags;
    driver->_reported_weight = msg.tank_weight;
}

void AC_SpotSprayer::run(const bool activate)
{
    // return immediately if no change
    if (_spraying == activate) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _spraying = _enabled && activate;

}

void AC_SpotSprayer::set_option(OPTION option)
{
    _option = option;
}



uint16_t AC_SpotSprayer::get_flow_rate()
{
    switch (_option)
    {
    case OPTION::LOW:
        return (uint16_t) _flow_rate_low;
    case OPTION::MIDDLE:
        return (uint16_t) _flow_rate_mid;
    case OPTION::HIGH:
        return (uint16_t) _flow_rate_high;
    default:
        return 0;
    }
}

void AC_SpotSprayer::queue_volume()
{
    if (_volume_queued == 0)
    {
        switch (_option)
        {
        case OPTION::LOW:
            if (_volume_low > 0)
            {
                _volume_queued = (uint16_t) _volume_low;
            }
            break;
        case OPTION::MIDDLE:
            if (_volume_mid > 0)
            {
                _volume_queued = (uint16_t) _volume_mid;
            }
            break;
        case OPTION::HIGH:
            if (_volume_high > 0)
            {
                _volume_queued = (uint16_t) _volume_high;
            }
            break;
        default:
            {
                break;
            }
        }
    }
}

void AC_SpotSprayer::request_pulse()
{
    if (_volume_queued == 0 && !_spraying)
    {
        _volume_queued = (uint16_t) _pulse;
    }
}

uint16_t  AC_SpotSprayer::volume_queued()
{
    uint16_t volume_to_send = _volume_queued;
    _volume_to_log = _volume_queued;
    _volume_queued = 0;
    return volume_to_send;
}

uint16_t AC_SpotSprayer::get_pressure()
{
    return (uint16_t) _pressure;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_SpotSprayer::update()
{
    if (!_enabled)
    {
        return;
    }
    // get the current time
    const uint32_t now = AP_HAL::millis();

    if (_mode == 1 && _spraying)
    {
        _spraying = false;
    }

    if (_last_fault_msg_ms + ERROR_MSG_TIMEOUT < now) {
        if (last_reading_ms + MSG_TIMEOUT < now) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Sprayer: No Communication");
            _last_fault_msg_ms = now;
            return;
        }

        if (error_flags > 0) {
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_FLOW_RATE_1)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: Possible blockage");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_FLOW_RATE_2)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: Nozzle 2 possible blockage");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_FLOW_RATE_3)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: Nozzle 3 possible blockage");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_FLOW_RATE_4)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: Nozzle 4 possible blockage");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_LOW_PRESSURE)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: Low Spray Pressure");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_OVER_PRESSURE)
            {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Sprayer: High Spray Pressure");
            }
            if (error_flags & COM_AERONAVICS_SPRAYINFO_ERROR_NO_SPRAY)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Sprayer: No spray remaining");
            }

            _last_fault_msg_ms = now;
        }

    }
#if HAL_LOGGING_ENABLED
    log_write();
#endif
}

bool AC_SpotSprayer::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (!_enabled)
    {
        return true;
    }
    if (_reported_weight > (float) _useful_load) {
        hal.util->snprintf(failmsg, failmsg_len, "Overweight by %fkg", _reported_weight - (float) _useful_load);
        return false;
    }
    return true;
}

#if HAL_LOGGING_ENABLED
void AC_SpotSprayer::log_write() 
{
#define MASK_LOG_ANY                    0xFFFF
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }
    if (last_logged_reading_ms == last_reading_ms) {
        return;
    }

    last_logged_reading_ms = last_reading_ms;

    uint16_t desired_flow_rate;
    if (_spraying)
    {
        desired_flow_rate = get_flow_rate();
    }
    else
    {
        desired_flow_rate = 0;
    }

    WITH_SEMAPHORE(_sem);
    AP::logger().WriteStreaming(
        "SPRAY",
        "TimeUS,DFlow,MFlow,DVol,Pres,SLevel,SVol,Error",
        "syylP%l-",
        "F-------",
        "QfffHffB",
        AP_HAL::micros64(),
        ((float)desired_flow_rate)/1000,
        ((float)measured_flow_rate)/1000,
        ((float)_volume_to_log)/1000,
        measured_pressure,
        spray_level,
        armed_sprayed_volume,
        error_flags
    );
}
#endif

void AC_SpotSprayer::send_spray_status(const mavlink_channel_t channel)
{
    if (last_reading_ms == 0)
    {
        // nothing to report
        return;
    }

    uint16_t desired_flow_rate;
    if (_spraying)
    {
        desired_flow_rate = get_flow_rate();
    }
    else
    {
        desired_flow_rate = 0;
    }

    mavlink_msg_anv_msg_spray_status_send(
        channel,
        measured_flow_rate,
        desired_flow_rate,
        total_sprayed_volume,
        armed_sprayed_volume,
        last_tree_volume,
        spray_level,
        measured_pressure,
        error_flags
    );
}

namespace AP {

AC_SpotSprayer *spot_sprayer()
{
    return AC_SpotSprayer::get_singleton();
}

};
#endif // HAL_SPRAYER_ENABLED
