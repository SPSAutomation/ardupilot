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

#include "AP_Generator_GX_16.h"

#if AP_GENERATOR_GX_16_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// init method; configure communications with the generator
void AP_Generator_GX_16::init()
{
    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = true;

}


//links the rangefinder uavcan message to this backend
void AP_Generator_GX_16::subscribe_msgs(AP_DroneCAN* ap_dronecan)
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
AP_Generator_GX_16* AP_Generator_GX_16::get_dronecan_backend(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    AP_Generator_GX_16* driver = nullptr;
    AP_Generator &frontend = *AP::generator();

    driver = (AP_Generator_GX_16*)frontend._driver_ptr;
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

//Generator message handler
void AP_Generator_GX_16::handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_GX16ExtenderInfo &msg)
{
    AP_Generator_GX_16* driver = get_dronecan_backend(ap_dronecan);
    if (driver == nullptr)
    {
        return;
    }

    WITH_SEMAPHORE(driver->_sem);
    //fetch the matching uavcan driver, node id and sensor id backend instance
    driver->last_reading_ms = AP_HAL::millis();
    driver->working_state = msg.WorkingState;
    driver->coolant_temp_1 = msg.ECT1 - 40;
    driver->coolant_temp_2 = msg.ECT2 - 40;
    driver->coil_temp = msg.GWT - 40;
    driver->engine_speed = msg.EngineSpeed;
    driver->output_voltage = ((float) msg.OutputVoltage) / 100;
    driver->output_current = (((float) msg.OutputCurrent) / 20) - 400;
    driver->battery_current = (((float) msg.BatteryCurrent) / 20) - 400;
    driver->target_throttle_position = ((float) msg.TargetTP1) / 20;
    driver->actual_throttle_position = ((float) msg.ActualTP1) / 20;
    driver->baro = msg.BARO;
    driver->IAT = msg.IAT - 40;
    driver->fuel_consumption = msg.FuelConsumption;
    driver->fuel_level = msg.GPS;
    driver->rail_12V = ((float)msg.B12V) / 10;
    driver->rail_5V1 = ((float)msg.B5V1) / 20;
    driver->rail_7V4 = ((float)msg.B7V4) / 20;
    driver->rail_VBATT = ((float)msg.VBAT) / 50;
    driver->rail_VREF = ((float)msg.VREF) / 50;
    driver->EmgST0 = msg.EmgST0;
    driver->EmgST1 = msg.EmgST1;
    driver->ErrST0 = msg.ErrST0;
    driver->ErrST1 = msg.ErrST1;
    driver->ErrST2 = msg.ErrST2;
    driver->ErrST3 = msg.ErrST3;
    driver->AlmST0 = msg.AlmST0;
    driver->AlmST1 = msg.AlmST1;
    driver->AlmST2 = msg.AlmST2;
    driver->AlmST3 = msg.AlmST3;
    driver->SysST0 = msg.SysST0;
    driver->SysST1 = msg.SysST1;
    driver->SysST2 = msg.SysST2;
    driver->RxCounter = msg.RxCounter;
    driver->TxCounter = msg.TxCounter;
    driver->BusError = msg.BusError;
    driver->TxAccTime = msg.TxAccTime;
    driver->RxCtrlAccTime = msg.RxCtrlAccTime;
    driver->RxCtrlAccCount = msg.RxCtrlAccCount;
    driver->RxMotorAccTime = msg.RxMotorAccTime;
    driver->RxMotorAccCount = msg.RxMotorAccCount;
}

// Fan status message handler
void AP_Generator_GX_16::handle_fans(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_aeronavics_FanStatus &msg)
{
    AP_Generator_GX_16* driver = get_dronecan_backend(ap_dronecan);
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
bool AP_Generator_GX_16::generator_ok_to_run() const
{
    return (coolant_temp_1 >= EXTENDER_PREARM_TEMP) && (coolant_temp_2 >= EXTENDER_PREARM_TEMP);
}

/*
  update the state of the sensor
*/
void AP_Generator_GX_16::update(void)
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
void AP_Generator_GX_16::update_runstate()
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
void AP_Generator_GX_16::Log_Write()
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
        "TimeUS,MsgUS,Rpm,Thr,Fuel,CoilT,M1Temp,M2Temp,Volt,Curr,State",
        "ssq%%OOOvA-",
        "FC---------",
        "QIHfBfffffB",
        AP_HAL::micros64(),
        last_logged_reading_ms,
        engine_speed,
        actual_throttle_position,
        fuel_level,
        coil_temp,
        coolant_temp_1,
        coolant_temp_2,
        output_voltage,
        output_current,
        working_state
    );

    AP::logger().WriteStreaming(
        "GEN2",
        "TimeUS,TThr,IAT,BCurr,R12V,R5V1,R7V4,VBATT,VREF",
        "s%OAvvvvv",
        "F--------",
        "Qffffffff",
        AP_HAL::micros64(),
        target_throttle_position,
        IAT,
        battery_current,
        rail_12V,
        rail_5V1,
        rail_7V4,
        rail_VBATT,
        rail_VREF
    );

    AP::logger().WriteStreaming(
        "GEN3",
        "TimeUS,EmgST0,EmgST1,ErrST0,ErrST1,ErrST2,ErrST3",
        "s------",
        "F------",
        "QBBBBBB",
        AP_HAL::micros64(),
        EmgST0,
        EmgST1,
        ErrST0,
        ErrST1,
        ErrST2,
        ErrST3
    );

    AP::logger().WriteStreaming(
        "GEN4",
        "TimeUS,AlmST0,AlmST1,AlmST2,AlmST3,SysST0,SysST1,SysST2",
        "s-------",
        "F-------",
        "QBBBBBBB",
        AP_HAL::micros64(),
        AlmST0,
        AlmST1,
        AlmST2,
        AlmST3,
        SysST0,
        SysST1,
        SysST2
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
bool AP_Generator_GX_16::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_reading_ms > 5000) { // we expect @1Hz
        hal.util->snprintf(failmsg, failmsg_len, "no messages in %ums", unsigned(now - last_reading_ms));
        return false;
    }


    if (
        (AlmST0 & (uint8_t)AbnormalAlert0::OUTPUT_OVER_CURRENT_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_CURRENT_ERROR)
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Over Current Error");
        return false;
    }
    if (
        AlmST2 & (uint8_t)AbnormalAlert2::LOW_OUTPUT_VOLTAGE_ALARM
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Low Voltage Error");
        return false;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::COIL_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COIL_OVER_TEMP_ALARM
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Coil Overtemp: %f°c", coil_temp);
        return false;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::COOL1_OVER_TEMP
        || EmgST0 & (uint8_t)OverlimitFault1::COOL2_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL1_OVER_TEMP_ALARM
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL2_OVER_TEMP_ALARM
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Motor Overtemp: %f°c", MAX(coolant_temp_1, coolant_temp_2));
        return false;
    }

    if (
        AlmST0 & (uint8_t)AbnormalAlert0::OVER_VOLTAGE_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_VOLTAGE_ERROR
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Over Voltage Error");
        return false;
    }

    if (
        AlmST2 & (uint8_t)AbnormalAlert2::THROTTLE_ABNORMAL_ALARM
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Throttle Error");
        return false;
    }

    if (
        EmgST1 & (uint8_t)OverlimitFault2::ENG_OVER_SPEED
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Over Speed Error");
        return false;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::INTAKE_OVER_TEMP
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Intake Overtemp: %f°c", IAT);
        return false;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::RAIL_OVER_TEMP
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Rail Overtemp");
        return false;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::BATT_CHRG_OVER_LIMIT
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Battery Overcurrent");
        return false;
    }

    if (
        EmgST1 & (uint8_t)OverlimitFault2::IGNITION_COIL_ERROR
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Ignition Coil Error");
        return false;
    }

    if (
        EmgST1 & (uint8_t)OverlimitFault2::FUEL_INJECTOR_ERROR
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Fuel Injector Error");
        return false;
    }

    if (
        EmgST1 & (uint8_t)OverlimitFault2::OIL_PUMP_ERROR
    ) {
        hal.util->snprintf(failmsg, failmsg_len, "Oil Pump Error");
        return false;
    }

    
    if (!(working_state & (uint8_t)WorkingState::RUN && working_state & (uint8_t)WorkingState::CRANK && working_state & (uint8_t)WorkingState::IDLE)) 
    {
        hal.util->snprintf(failmsg, failmsg_len, "Not Started");
        return false;
    }

    if (coolant_temp_1 < EXTENDER_PREARM_TEMP)
    {
        hal.util->snprintf(failmsg, failmsg_len, "Warming up: %f Needs %u", coolant_temp_1, EXTENDER_PREARM_TEMP);
        return false;
    }

    if (coolant_temp_2 < EXTENDER_PREARM_TEMP)
    {
        hal.util->snprintf(failmsg, failmsg_len, "Warming up: %f Needs %u", coolant_temp_2, EXTENDER_PREARM_TEMP);
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
void AP_Generator_GX_16::update_frontend_readings(void)
{
    WITH_SEMAPHORE(_sem);
    _voltage = output_voltage;
    _current = output_current;
    _rpm = engine_speed;
    _fuel_remaining = ((float)fuel_level) / 100;
    _commanded_state = (uint8_t)commanded_runstate;

    if (working_state & (uint8_t)WorkingState::CRANK || working_state & (uint8_t)WorkingState::IDLE || working_state & (uint8_t)WorkingState::RUN)
    {
        _state = 2;
    }
    else 
    {
        _state = 0;
    }

    update_frontend();
}


// healthy returns true if the generator is not present, or it is
// present, providing telemetry and not indicating an errors.
bool AP_Generator_GX_16::healthy() const
{
    const uint32_t now = AP_HAL::millis();

    if (last_reading_ms == 0 || now - last_reading_ms > 5000) {
        if (now - last_error_sent > 5000)
        {
            last_error_sent = now;
            gcs().send_text(MAV_SEVERITY_WARNING, "Time since last generator message: %lums", now - last_reading_ms);
        }
        return false;
    }

    bool error_seen = false;
    bool message_sent = false;

    if (AlmST0 & (uint8_t)AbnormalAlert0::OUTPUT_OVER_CURRENT_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_CURRENT_ERROR) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Current Error");
            message_sent = true;
        }
        error_seen = true;
    }
    if (AlmST2 & (uint8_t)AbnormalAlert2::LOW_OUTPUT_VOLTAGE_ALARM) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Low Voltage Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST0 & (uint8_t)OverlimitFault1::COIL_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COIL_OVER_TEMP_ALARM) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Coil Overtemp: %f°c", coil_temp);
            message_sent = true;
        }
        error_seen = true;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::COOL1_OVER_TEMP
        || EmgST0 & (uint8_t)OverlimitFault1::COOL2_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL1_OVER_TEMP_ALARM
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL2_OVER_TEMP_ALARM
    ) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Motor Overtemp: %f°c", MAX(coolant_temp_1, coolant_temp_2));
            message_sent = true;
        }
        error_seen = true;
    }

    if (AlmST0 & (uint8_t)AbnormalAlert0::OVER_VOLTAGE_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_VOLTAGE_ERROR) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Voltage Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (AlmST2 & (uint8_t)AbnormalAlert2::THROTTLE_ABNORMAL_ALARM) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Throttle Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST1 & (uint8_t)OverlimitFault2::ENG_OVER_SPEED) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Over Speed Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST0 & (uint8_t)OverlimitFault1::INTAKE_OVER_TEMP) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Intake Overtemp: %f°c", IAT);
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST0 & (uint8_t)OverlimitFault1::RAIL_OVER_TEMP) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Rail Overtemp");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST0 & (uint8_t)OverlimitFault1::BATT_CHRG_OVER_LIMIT) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Battery Overcurrent");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST1 & (uint8_t)OverlimitFault2::IGNITION_COIL_ERROR) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Ignition Coil Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST1 & (uint8_t)OverlimitFault2::FUEL_INJECTOR_ERROR) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Fuel Injector Error");
            message_sent = true;
        }
        error_seen = true;
    }

    if (EmgST1 & (uint8_t)OverlimitFault2::OIL_PUMP_ERROR) {
        if (now - last_error_sent > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Oil Pump Error");
            message_sent = true;
        }
        error_seen = true;
    }


    if (message_sent)
    {
        last_error_sent = now;
    }

    if (error_seen)
    {
        return false;
    }
    return true;
}

// Battery failsafe not used
AP_BattMonitor::Failsafe AP_Generator_GX_16::update_failsafes() const
{
    return AP_BattMonitor::Failsafe::None;
}

// Check for error codes that are deemed critical
bool AP_Generator_GX_16::is_critical_error(const uint32_t err_in) const
{
    return false;
}

// Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
bool AP_Generator_GX_16::is_low_error(const uint32_t err_in) const
{
    return false;
}


// send mavlink generator status
void AP_Generator_GX_16::send_generator_status(const GCS_MAVLINK &channel)
{
    if (last_reading_ms == 0) {
        // nothing to report
        return;
    }
    WITH_SEMAPHORE(_sem);

    uint64_t status = 0;

    if (working_state & (uint8_t)WorkingState::CRANK 
        || working_state & (uint8_t)WorkingState::IDLE
        || working_state & (uint8_t)WorkingState::RUN
    )
    {
        if (!generator_ok_to_run()) {
            status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
        } else {
            if (AP::arming().is_armed()) {
                status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
            } else {
                status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
            }
        }
    }
    else
    {
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
    }

    if (commanded_runstate == RunState::STOP) {
        status |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }


    if (
        AlmST0 & (uint8_t)AbnormalAlert0::OUTPUT_OVER_CURRENT_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_CURRENT_ERROR
    ) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT;
    }
    if (
        AlmST2 & (uint8_t)AbnormalAlert2::LOW_OUTPUT_VOLTAGE_ALARM
    ) {
        status |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::COIL_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COIL_OVER_TEMP_ALARM
    ) {
        status |= MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING;
    }

    if (
        EmgST0 & (uint8_t)OverlimitFault1::COOL1_OVER_TEMP
        || EmgST0 & (uint8_t)OverlimitFault1::COOL2_OVER_TEMP
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL1_OVER_TEMP_ALARM
        || AlmST0 & (uint8_t)AbnormalAlert0::COOL2_OVER_TEMP_ALARM
    ) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING;
    }

    if (
        AlmST0 & (uint8_t)AbnormalAlert0::OVER_VOLTAGE_ALARM
        || EmgST0 & (uint8_t)OverlimitFault1::OVER_VOLTAGE_ERROR
    ) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
        engine_speed, // generator_speed
        battery_current, // battery_current; current into/out of battery
        output_current, // load_current; Current going to UAV
        output_current * output_voltage, // power_generated; the power being generated
        output_voltage, // bus_voltage; Voltage of the bus seen at the generator
        coil_temp, // rectifier_temperature
        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
        MAX(coolant_temp_1, coolant_temp_2), // generator temperature
        0, // total runtime in seconds
        0 // Time till next service in seconds
        );

    mavlink_msg_fuel_status_send(
        channel.get_chan(),
        0,  // Fuel ID.
        _frontend.get_fuel_tank_size(), // total fuel capacity
        std::numeric_limits<float>::quiet_NaN(),  // Consumed fuel (measured). 
        std::numeric_limits<float>::quiet_NaN(),  // Remaining fuel until empty (measured).
        fuel_level,  // Percentage of remaining fuel, relative to full.
        std::numeric_limits<float>::quiet_NaN(),  // Flow rate. Positive value when emptying/using, and negative if filling/replacing.
        std::numeric_limits<float>::quiet_NaN(),  // Fuel temperature. 
        MAV_FUEL_TYPE_LIQUID  // Fuel type. 
    );


    // const uint32_t now = AP_HAL::millis();
    //
    // if (((_frontend.get_last_service_time() * 3600) + EXTENDER_MAINTENANCE_SCHEDULE - (total_run_time * 60) <= 0) && now - last_maintenance_warning_ms > 30000)
    // {
    //     last_maintenance_warning_ms = now;
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Aircraft Requires Service");
    // }
}

// methods to control the generator state:
bool AP_Generator_GX_16::stop() 
{
    set_pilot_desired_runstate(RunState::STOP); 
    return true;
}

bool AP_Generator_GX_16::idle()
{
    set_pilot_desired_runstate(RunState::IDLE);
    return true;
}

bool AP_Generator_GX_16::run()
{
    set_pilot_desired_runstate(RunState::RUN);
    return true;
}
#endif  // AP_GENERATOR_RICHENPOWER_ENABLED
