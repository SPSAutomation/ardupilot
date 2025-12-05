/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Generator.h"

#if HAL_GENERATOR_ENABLED

#include "AP_Generator_IE_650_800.h"
#include "AP_Generator_IE_2400.h"
#include "AP_Generator_RichenPower.h"
#include "AP_Generator_GX_7.h"
#include "AP_Generator_GX_16.h"

#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Generator::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Generator type
    // @Description: Generator type
    // @Values: 0:Disabled, 1:IE 650w 800w Fuel Cell, 2:IE 2.4kW Fuel Cell, 3: Richenpower, 4: GX-7
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Generator, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: OPTIONS
    // @DisplayName: Generator Options
    // @Description: Bitmask of options for generators
    // @Bitmask: 0:Suppress Maintenance-Required Warnings
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 2, AP_Generator, _options, 0),

    // @Param: LOW_PER
    // @DisplayName: Generator low fuel level
    // @Description: Fuel level to low failsafe at
    // Units: %
    // @User: Standard
    AP_GROUPINFO("LOW_PER", 3, AP_Generator, _fuel_low_percent, 0),

    // @Param: LOW_FS
    // @DisplayName: Generator low fuel action
    // @Description: Action to take when low fuel level is reached
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @User: Standard
    AP_GROUPINFO("LOW_FS", 4, AP_Generator, _fuel_low_fs, 0),

    // @Param: CRIT_PER
    // @DisplayName: Generator critical fuel level
    // @Description: Fuel level to critical failsafe at
    // Units: %
    // @User: Standard
    AP_GROUPINFO("CRIT_PER", 5, AP_Generator, _fuel_crit_percent, 0),

    // @Param: CRIT_FS
    // @DisplayName: Generator critical fuel action
    // @Description: Action to take when critical fuel level is reached
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @User: Standard
    AP_GROUPINFO("CRIT_FS", 6, AP_Generator, _fuel_crit_fs, 0),

    // @Param: OFF_FS
    // @DisplayName: Generator stopped action
    // @Description: Action to take when generator stops
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @User: Standard
    AP_GROUPINFO("OFF_FS", 7, AP_Generator, _off_fs, 0),

    // @Param: ERROR_FS
    // @DisplayName: Generator error action
    // @Description: Action to take when generator has an error
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Terminate,6:Auto DO_LAND_START or RTL
    // @User: Standard
    AP_GROUPINFO("ERROR_FS", 8, AP_Generator, _error_fs, 0),

    // @Param: LAST_SERV
    // @DisplayName: Generator Last Service Time
    // @Description: Generator Runtime in hours at last service
    // @Units: h
    // @User: Standard
    AP_GROUPINFO("LAST_SERV", 9, AP_Generator, _last_service_time, 0),

    // @Param: MIN_ARM_LVL
    // @DisplayName: Generator Prearm Fuel Level
    // @Description: Fuel level to prearm fail at
    // Units: %
    // @User: Standard
    AP_GROUPINFO("MIN_ARM_LVL", 10, AP_Generator, _fuel_prearm_percent, 0),

    // @Param:  TANK_SIZE
    // @DisplayName: Generator Fuel Tank Size
    // @Description: Fuel Tank size in Litres
    // Units: L
    // @User: Standard
    AP_GROUPINFO("TANK_SIZE", 11, AP_Generator, _fuel_tank_size, 0),

    AP_GROUPEND
};

// Constructor
AP_Generator::AP_Generator()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many generators");
#endif
        return;
    }
    _singleton = this;
}

void AP_Generator::init()
{
    // Select backend
    switch (type()) {
        case Type::GEN_DISABLED:
            // Not using a generator
            return;

#if AP_GENERATOR_IE_650_800_ENABLED
        case Type::IE_650_800:
            _driver_ptr = new AP_Generator_IE_650_800(*this);
            break;
#endif

#if AP_GENERATOR_IE_2400_ENABLED
        case Type::IE_2400:
            _driver_ptr = new AP_Generator_IE_2400(*this);
            break;
#endif

#if AP_GENERATOR_RICHENPOWER_ENABLED
        case Type::RICHENPOWER:
            _driver_ptr = new AP_Generator_RichenPower(*this);
            break;
#endif
#if AP_GENERATOR_GX_7_ENABLED
        case Type::GX_7:
            _driver_ptr = new AP_Generator_GX_7(*this);
            break;
#endif
#if AP_GENERATOR_GX_16_ENABLED
        case Type::GX_16:
            _driver_ptr = new AP_Generator_GX_16(*this);
            break;
#endif
    }

    if (_driver_ptr != nullptr) {
        _driver_ptr->init();
    }
}

void AP_Generator::update()
{
    // Return immediately if not enabled. Don't support run-time disabling of generator
    if (_driver_ptr == nullptr) {
        return;
    }

    // Calling backend update will cause backend to update the front end variables
    _driver_ptr->update();
}

// Helper to get param and cast to Type
enum AP_Generator::Type AP_Generator::type() const
{
    return (Type)_type.get();
}

// Pass through to backend
void AP_Generator::send_generator_status(const GCS_MAVLINK &channel)
{
    if (_driver_ptr == nullptr) {
        return;
    }
    _driver_ptr->send_generator_status(channel);
}

// Tell backend to perform arming checks
bool AP_Generator::pre_arm_check(char* failmsg, uint8_t failmsg_len) const
{
    if (type() == Type::GEN_DISABLED) {
        // Don't prevent arming if generator is not enabled and has never been init
        if (_driver_ptr == nullptr) {
            return true;
        }
        // Don't allow arming if we have disabled the generator since boot
        strncpy(failmsg, "Generator disabled, reboot required", failmsg_len);
        return false;
    }
    if (_driver_ptr == nullptr) {
        strncpy(failmsg, "No backend driver", failmsg_len);
        return false;
    }
    return _driver_ptr->pre_arm_check(failmsg, failmsg_len);
}

// Tell backend check failsafes
AP_BattMonitor::Failsafe AP_Generator::update_failsafes()
{
    // Don't invoke a failsafe if driver not assigned
    if (_driver_ptr == nullptr) {
        return AP_BattMonitor::Failsafe::None;
    }
    return _driver_ptr->update_failsafes();
}

// Pass through to backend
bool AP_Generator::stop()
{
    // Still allow 
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->stop();
}

// Pass through to backend
bool AP_Generator::idle()
{
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->idle();
}

// Pass through to backend
bool AP_Generator::run()
{
    // Don't allow operators to request generator be set to run if it has been disabled
    if (_driver_ptr == nullptr) {
        return false;
    }
    return _driver_ptr->run();
}

// Pass through to backend
void AP_Generator::shutdown_on_land(bool shutdown)
{
    // Don't allow operators to request generator be set to run if it has been disabled
    if (_driver_ptr == nullptr) {
        return;
    }
    _driver_ptr->shutdown_on_land(shutdown);
}

// Get the AP_Generator singleton
AP_Generator *AP_Generator::get_singleton()
{
    return _singleton;
}

AP_Generator *AP_Generator::_singleton = nullptr;

namespace AP {
    AP_Generator *generator()
    {
        return AP_Generator::get_singleton();
    }
};
#endif
