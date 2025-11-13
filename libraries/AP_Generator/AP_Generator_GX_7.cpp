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

#include "AP_Generator_GX_7.h"

#if AP_GENERATOR_GX_7_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// init method; configure communications with the generator
void AP_Generator_GX_7::init()
{
    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = true;

}


//links the rangefinder uavcan message to this backend
void AP_Generator_GX_7::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    get_dronecan_backend(ap_dronecan);

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_measurement, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("measurement_sub");
    }
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_fans, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("measurement_sub");
    }
}

//Method to find the backend relating to the node id
AP_Generator_GX_7* AP_Generator_GX_7::get_dronecan_backend(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    AP_Generator_GX_7* driver = nullptr;
    AP_Generator &frontend = *AP::generator();

    driver = (AP_Generator_GX_7*)frontend._driver_ptr;
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

// GX7 generator info message handler
void AP_Generator_GX_7::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_ExtenderInfo &msg)
{
    AP_Generator_GX_7* driver = get_dronecan_backend(ap_dronecan);
    if (driver == nullptr)
    {
        return;
    }

    WITH_SEMAPHORE(driver->_sem);
    //fetch the matching uavcan driver, node id and sensor id backend instance
    driver->last_reading_ms = AP_HAL::millis();
    driver->engine_speed = msg.EngineSpeed;
    driver->throttle_position = msg.ThrottlePosition;
    driver->fuel_level = msg.FuelPosition;
    driver->coil_temperature = msg.Motortemperature - 80;
    driver->cylinder_temperature = msg.EngineCylinderTemperature - 40;
    driver->output_voltage = msg.OutputVoltage;
    driver->output_current = msg.OutputCurrent;
    driver->total_run_time = msg.total_run_minutes;
    driver->extender_error = msg.ExtenderAlarm;
    driver->working_state = (WorkingState)msg.WorkingState;
}

// Fan status message handler
void AP_Generator_GX_7::handle_fans(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_FanStatus &msg)
{
    AP_Generator_GX_7* driver = get_dronecan_backend(ap_dronecan);
    if (driver == nullptr)
    {
        return;
    }

    WITH_SEMAPHORE(driver->_sem);
    //fetch the matching uavcan driver, node id and sensor id backend instance

    while (msg.fan_index >= driver->fanInfo.size())
    {
        fanStatus fan;
        fan.id = driver->fanInfo.size() - 1;
        driver->fanInfo.push_back(fan);
    }

    driver->fanInfo[msg.fan_index].rpm = msg.rpm;
    driver->fanInfo[msg.fan_index].power_pct = msg.power_pct;
    driver->fanInfo[msg.fan_index].health = msg.health;
}

// returns true if the generator should be allowed to move into
// the "run" (high-RPM) state:
bool AP_Generator_GX_7::generator_ok_to_run() const
{
    return cylinder_temperature >= EXTENDER_PREARM_TEMP;
}

/*
  update the state of the sensor
*/
void AP_Generator_GX_7::update(void)
{
    update_runstate();

    update_frontend_readings();

#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
}

// update_runstate updates the servo output we use to control the
// generator.  Which state we request the generator move to depends on
// the RC inputcontrol and the temperature the generator is at.
void AP_Generator_GX_7::update_runstate()
{
    // if the vehicle crashes then we assume the pilot wants to stop
    // the motor.  This is done as a once-off when the crash is
    // detected to allow the operator to rearm the vehicle, or we end
    // up in a catch-22 situation where we force the stop state on the
    // generator so they can't arm and can't start the generator
    // because the vehicle is crashed.
    if (AP::vehicle()->is_crashed()) {
        if (!vehicle_was_crashed) {
            gcs().send_text(MAV_SEVERITY_INFO, "Crash; stopping generator");
            pilot_desired_runstate = RunState::STOP;
            vehicle_was_crashed = true;
        }
    } else {
        vehicle_was_crashed = false;
    }

    if (shutdown_on_landing && !AP::arming().is_armed())
    {
        pilot_desired_runstate = RunState::STOP;
    }

    if (commanded_runstate != pilot_desired_runstate &&
        !hal.util->get_soft_armed()) {
        // consider changing the commanded runstate to the pilot
        // desired runstate:
        switch (pilot_desired_runstate) {
        case RunState::STOP:
            commanded_runstate = pilot_desired_runstate;
            break;
        case RunState::IDLE:
        case RunState::RUN:
            commanded_runstate = RunState::RUN;
            break;
        }
    }
}

#if HAL_LOGGING_ENABLED
// log generator status to the onboard log
void AP_Generator_GX_7::Log_Write()
{
#define MASK_LOG_ANY                    0xFFFF
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }
    if (last_logged_reading_ms == last_reading_ms) {
        return;
    }
    last_logged_reading_ms = last_reading_ms;

    WITH_SEMAPHORE(_sem);
    AP::logger().WriteStreaming(
        "GEN",
        "TimeUS,MsgUS,Rpm,Thr,Fuel,GTemp,MTemp,Volt,Curr,Runtime,Err,State",
        "ssq%%OOvAs--",
        "FC----------",
        "QIHHBBBHHIIB",
        AP_HAL::micros64(),
        last_logged_reading_ms,
        engine_speed,
        throttle_position,
        fuel_level,
        coil_temperature,
        cylinder_temperature,
        output_voltage,
        output_current,
        total_run_time*60,
        extender_error,
        working_state
        );
        
    for (uint8_t i = 0; i < fanInfo.size(); i++)
    {
        char fan_log_name[4];
        hal.util->snprintf(fan_log_name, 4, "FAN%d", i);
        
        AP::logger().WriteStreaming(
        fan_log_name,
        "TimeUS,Rpm,Thr,Health",
        "sq%-",
        "F---",
        "QHHB",
        AP_HAL::micros64(),
        fanInfo[i].rpm,
        fanInfo[i].power_pct,
        fanInfo[i].health
        );
    }
}
#endif

// generator prearm checks; notably, if we never see a generator we do
// not run the checks.  Generators are attached/detached at will, and
// reconfiguring is painful.
bool AP_Generator_GX_7::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_reading_ms > 5000) { // we expect @1Hz
        hal.util->snprintf(failmsg, failmsg_len, "no messages in %ums", unsigned(now - last_reading_ms));
        return false;
    }

    uint32_t errors = extender_error;

    // maintenance is a prearm error, but we will ignore the built in 
    // maintenance error and use our own one as we cannot reset the
    // built in one after a service
    errors &= ~(1U << uint32_t(ExtenderError::MAINTENANCE_TIME_ERROR));

    if (errors) {

        for (uint8_t i=0; i<32; i++) {
            if (errors & (1U << i)) {
                if (errors < (uint32_t)ExtenderError::LAST) {
                    hal.util->snprintf(failmsg, failmsg_len, "error: %s", error_strings[i]);
                } else {
                    hal.util->snprintf(failmsg, failmsg_len, "unknown error: 1U<<%u", i);
                }
            }
        }
        return false;
    }
    
    if (working_state != WorkingState::RUN) {
        hal.util->snprintf(failmsg, failmsg_len, "Not Started");
        return false;
    }

    if (cylinder_temperature < EXTENDER_PREARM_TEMP)
    {
        hal.util->snprintf(failmsg, failmsg_len, "Warming up: %u Needs %u", cylinder_temperature, EXTENDER_PREARM_TEMP);
        return false;
    }

    if (fuel_level < _frontend.get_prearm_fuel_level())
    {
        hal.util->snprintf(failmsg, failmsg_len, "Low Fuel: %u%% less than %u%%", fuel_level, _frontend.get_prearm_fuel_level());
        return false;
    }

    return true;
}

// Update front end with voltage, current, and rpm values
void AP_Generator_GX_7::update_frontend_readings(void)
{
    WITH_SEMAPHORE(_sem);
    _voltage = output_voltage;
    _current = output_current;
    _rpm = engine_speed;
    _fuel_remaining = ((float)fuel_level) / 100;
    _state = (uint8_t)working_state;
    _commanded_state = (uint8_t)commanded_runstate;

    update_frontend();
}


// healthy returns true if the generator is not present, or it is
// present, providing telemetry and not indicating an errors.
bool AP_Generator_GX_7::healthy() const
{
    const uint32_t now = AP_HAL::millis();

    if (last_reading_ms == 0 || now - last_reading_ms > 5000) {
        if (now - last_error_sent > 5000)
        {
            last_error_sent = now;
            gcs().send_text(MAV_SEVERITY_WARNING, "Time since last generator message: %ldms", now - last_reading_ms);
        }
        return false;
    }
    if (extender_error & (~(uint32_t)ExtenderError::COMMUNICATION_ERROR)) {
        if (now - last_error_sent > 5000)
        {
            last_error_sent = now;
            if (extender_error & (uint32_t)ExtenderError::MAINTENANCE_TIME_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Requires Maintenance");
            }
            if (extender_error & (uint32_t)ExtenderError::SYSTEM_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator System Error");
            }
            // if (extender_error & (uint32_t)ExtenderError::COMMUNICATION_ERROR)
            // {
            //     gcs().send_text(MAV_SEVERITY_WARNING, "Generator Communication Error");
            // }
            if (extender_error & (uint32_t)ExtenderError::COIL_OVER_TEMP_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Coil Overtemp: %u°c", coil_temperature);
            }
            if (extender_error & (uint32_t)ExtenderError::COOLANT_OVER_TEMP_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Motor Overtemp: %u°c", cylinder_temperature);
            }
            if (extender_error & (uint32_t)ExtenderError::THROTTLE_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Throttle Error");
            }
            if (extender_error & (uint32_t)ExtenderError::OVER_SPEED_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Speed Error");
            }
            if (extender_error & (uint32_t)ExtenderError::OVER_CURRENT_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Current Error");
            }
            if (extender_error & (uint32_t)ExtenderError::LOW_VOLTAGE_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Low Voltage Error");
            }
            if (extender_error & (uint32_t)ExtenderError::OVER_VOLTAGE_ERROR)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Voltage Error");
            }
        }

        return false;
    }
    return true;
}

// Check for failsafes
AP_BattMonitor::Failsafe AP_Generator_GX_7::update_failsafes() const
{
    // Check for error codes that lead to critical action battery monitor failsafe
    if (is_critical_error(extender_error)) {
        return AP_BattMonitor::Failsafe::Critical;
    }

    // Check for error codes that lead to low action battery monitor failsafe
    if (is_low_error(extender_error)) {
        return AP_BattMonitor::Failsafe::Low;
    }

    return AP_BattMonitor::Failsafe::None;
}

// Check for error codes that are deemed critical
bool AP_Generator_GX_7::is_critical_error(const uint32_t err_in) const
{
    return false;
}

// Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
bool AP_Generator_GX_7::is_low_error(const uint32_t err_in) const
{
    return false;
}


// TODO update this function
// send mavlink generator status
void AP_Generator_GX_7::send_generator_status(const GCS_MAVLINK &channel)
{
    if (last_reading_ms == 0) {
        // nothing to report
        return;
    }
    WITH_SEMAPHORE(_sem);

    uint64_t status = 0;
    if (engine_speed == 0) {
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
    } else {
        switch (working_state) {
        case WorkingState::STOP:
            status |= MAV_GENERATOR_STATUS_FLAG_OFF;
            break;
        case WorkingState::RUN:
            if (!generator_ok_to_run()) {
                status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
            } else {
                if (AP::arming().is_armed()) {
                    status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
                }
                else {
                    status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
                }
            }
            break;
        default:
            break;
        }
    }

    if (commanded_runstate != RunState::RUN) {
        status |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }


    if (extender_error & (uint8_t)ExtenderError::OVER_CURRENT_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT;
    }
    if (extender_error & (uint8_t)ExtenderError::LOW_VOLTAGE_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
    }

    if (extender_error & (uint8_t)ExtenderError::MAINTENANCE_TIME_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED;
    }

    if (extender_error & (uint8_t)ExtenderError::COMMUNICATION_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING;
    }

    if (extender_error & (uint8_t)ExtenderError::COIL_OVER_TEMP_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING;
    }

    if (extender_error & (uint8_t)ExtenderError::COOLANT_OVER_TEMP_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING;
    }

    if (extender_error & (uint8_t)ExtenderError::OVER_VOLTAGE_ERROR) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
        engine_speed, // generator_speed
        std::numeric_limits<double>::quiet_NaN(), // battery_current; current into/out of battery
        output_current, // load_current; Current going to UAV
        output_current * output_voltage, // power_generated; the power being generated
        output_voltage, // bus_voltage; Voltage of the bus seen at the generator
        coil_temperature, // rectifier_temperature
        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
        cylinder_temperature, // generator temperature
        total_run_time * 60, // total runtime in seconds
        (_frontend.get_last_service_time() * 3600) + EXTENDER_MAINTAINANCE_SCHEDULE - (total_run_time * 60) // Time till next service in seconds
        );

    mavlink_msg_fuel_status_send(
        channel.get_chan(),
        0,  // Fuel ID.
        5000, // total fuel capacity
        std::numeric_limits<float>::quiet_NaN(),  // Consumed fuel (measured). 
        std::numeric_limits<float>::quiet_NaN(),  // Remaining fuel until empty (measured).
        fuel_level,  // Percentage of remaining fuel, relative to full.
        std::numeric_limits<float>::quiet_NaN(),  // Flow rate. Positive value when emptying/using, and negative if filling/replacing.
        std::numeric_limits<float>::quiet_NaN(),  // Fuel temperature. 
        MAV_FUEL_TYPE_LIQUID  // Fuel type. 
    );


    const uint32_t now = AP_HAL::millis();

    if (((_frontend.get_last_service_time() * 3600) + EXTENDER_MAINTAINANCE_SCHEDULE - (total_run_time * 60) <= 0) && now - last_maintenance_warning_ms > 30000)
    {
        last_maintenance_warning_ms = now;
        gcs().send_text(MAV_SEVERITY_WARNING, "Aircraft Requires Service");
    }
}

// methods to control the generator state:
bool AP_Generator_GX_7::stop() 
{
    set_pilot_desired_runstate(RunState::STOP); 
    return true;
}

bool AP_Generator_GX_7::idle()
{
    set_pilot_desired_runstate(RunState::IDLE);
    return true;
}

bool AP_Generator_GX_7::run()
{
    set_pilot_desired_runstate(RunState::RUN);
    return true;
}
#endif  // AP_GENERATOR_RICHENPOWER_ENABLED
