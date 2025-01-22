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
#include <map>


#define BOOM_MSG_TIMEOUT                         1000

#define BOOM_ERROR_MSG_TIMEOUT                   10000


/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class AC_BoomLock {
public:
    AC_BoomLock();

    /* Do not allow copies */
    CLASS_NO_COPY(AC_BoomLock);

    static AC_BoomLock *get_singleton();
    static AC_BoomLock *_singleton;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void handle_boom_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_BoomStatus &msg);
    static AC_BoomLock* get_dronecan_backend(AP_DroneCAN* ap_dronecan);

    bool enabled() const { return (bool) _enabled; }

    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Int8         _enabled;               ///< Top level enable/disable control
    AP_Int8         _num_booms;             ///< Number of booms to check locks for

private:


    // internal variables

    uint32_t    _last_fault_msg_ms;
    
    std::map<uint8_t, uint32_t> _boom_connection_status;

    HAL_Semaphore _sem;

    AP_DroneCAN* _ap_dronecan;

    bool _booms_connected;


};

namespace AP {
    AC_BoomLock *boom_lock();
};
