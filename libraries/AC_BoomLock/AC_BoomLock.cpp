#include "AC_BoomLock.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_BoomLock::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_BoomLock, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NUM
    // @DisplayName: Number of Booms
    // @Description: Number of Booms to check locks for
    // @Units: count
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("NUM",   1, AC_BoomLock, _num_booms, 0),

    AP_GROUPEND
};

AC_BoomLock::AC_BoomLock()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many Boom status checkers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < _num_booms; i++)
    {
        _boom_connection_status[i] = 0;
    }
}

/*
 * Get the AP_Sprayer singleton
 */
AC_BoomLock *AC_BoomLock::_singleton;
AC_BoomLock *AC_BoomLock::get_singleton()
{
    return _singleton;
}


//links the rangefinder uavcan message to this backend
void AC_BoomLock::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    get_dronecan_backend(ap_dronecan);

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_boom_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("measurement_sub");
    }
}

//Method to find the backend relating to the node id
AC_BoomLock* AC_BoomLock::get_dronecan_backend(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    AC_BoomLock* driver = AP::boom_lock();

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
void AC_BoomLock::handle_boom_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_BoomStatus &msg)
{
    AC_BoomLock* driver = get_dronecan_backend(ap_dronecan);
    if (driver == nullptr)
    {
        return;
    }

    WITH_SEMAPHORE(driver->_sem);
    uint32_t now = AP_HAL::millis();
    if (msg.connection_status)
    {
        driver->_boom_connection_status[msg.boom_id - 1] = now;
    }
    else
    {
        driver->_boom_connection_status[msg.boom_id - 1] = 0;
    }
}


/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_BoomLock::update()
{
    if (!_enabled)
    {
        return;
    }
    // get the current time
    const uint32_t now = AP_HAL::millis();

    bool all_connected = true;

    if (_last_fault_msg_ms + BOOM_ERROR_MSG_TIMEOUT < now) 
    {
        for (uint8_t i = 1; i <= _num_booms; i++)
        {
            if (_boom_connection_status[i - 1] + BOOM_MSG_TIMEOUT < now)
            {
                gcs().send_text(MAV_SEVERITY_ERROR, "Boom %u not connected", i+1);
                _last_fault_msg_ms = now;
                all_connected = false;
            }
        }
    }
    _booms_connected = all_connected;
}

bool AC_BoomLock::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (!_enabled)
    {
        return true;
    }
    if (!_booms_connected)
    {
        hal.util->snprintf(failmsg, failmsg_len, "Booms not connected");
        return false;
    }
    
    return true;
}

namespace AP {

AC_BoomLock *boom_lock()
{
    return AC_BoomLock::get_singleton();
}

};
