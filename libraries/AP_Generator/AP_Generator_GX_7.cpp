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
    // TODO get DroneCan interface

    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = true;

}


// returns true if the generator should be allowed to move into
// the "run" (high-RPM) state:
bool AP_Generator_GX_7::generator_ok_to_run() const
{
    return heat > heat_required_for_run();
}

// returns an amount of synthetic heat required for the generator
// to move into the "run" state:
constexpr float AP_Generator_GX_7::heat_required_for_run()
{
    // Return if motor is warmed up enough to start
    return motor_temperature >= start_temp;
}


void AP_Generator_GX_7::check_maintenance_required()
{
    // don't bother the user while flying:
    if (hal.util->get_soft_armed()) {
        return;
    }

    if (!AP::generator()->option_set(AP_Generator::Option::INHIBIT_MAINTENANCE_WARNINGS)) {
        const uint32_t now = AP_HAL::millis();

        if ((extender_error >> 1) & 0x1) {
            if (now - last_maintenance_warning_ms > 60000) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Generator: requires maintenance");
                last_maintenance_warning_ms = now;
            }
        }
    }
}

/*
  update the state of the sensor
*/
void AP_Generator_GX_7::update(void)
{
    if (last_reading_ms != 0) {
        update_runstate();
        check_maintenance_required();
    }

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
    // don't run the generator while the safety is on:
    // if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    //     _servo_channel->set_output_pwm(SERVO_PWM_STOP);
    //     return;
    // }

    static const uint16_t SERVO_PWM_STOP = 1200;
    static const uint16_t SERVO_PWM_IDLE = 1500;
    static const uint16_t SERVO_PWM_RUN = 1900;

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

    if (commanded_runstate != pilot_desired_runstate &&
        !hal.util->get_soft_armed()) {
        // consider changing the commanded runstate to the pilot
        // desired runstate:
        commanded_runstate = RunState::IDLE;
        switch (pilot_desired_runstate) {
        case RunState::STOP:
            // The H2 will keep the motor running for ~20 seconds for cool-down
            // we must go via the idle state or the H2 disallows moving to stop!
            if (time_in_idle_state_ms() > 1000) {
                commanded_runstate = pilot_desired_runstate;
            }
            break;
        case RunState::IDLE:
            // can always switch to idle
            commanded_runstate = pilot_desired_runstate;
            break;
        case RunState::RUN:
            // must have idled for a while before moving to run:
            if (generator_ok_to_run()) {
                commanded_runstate = pilot_desired_runstate;
            }
            break;
        }
    }

    switch (commanded_runstate) {
    case RunState::STOP:
        // TODO: Send DroneCan message to stop generator.
        break;
    case RunState::START:
    // TODO: Send DroneCan message to start generator.
        break;
    case RunState::IDLE:
        // TODO: Send DroneCan message to tell generator we are disarmed.
        break;
    case RunState::RUN:
        // TODO: Send DroneCan message to tell generator we are armed.
        break;
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

    AP::logger().WriteStreaming(
        "GEN",
        "TimeUS,rpm,throttle,fuel_level,temp,cyclinder_temp,ovolt,ocurr,runtime,error,state",
        "s-------",
        "F-------",
        "QIIHHffB",
        AP_HAL::micros64(),
        engine_speed,
        throttle_position,
        fuel_level,
        motor_temperature,
        engine_cyclinder_temperature,
        output_voltage,
        output_current,
        total_run_time,
        extender_error,
        working_state
        );
}
#endif

// generator prearm checks; notably, if we never see a generator we do
// not run the checks.  Generators are attached/detached at will, and
// reconfiguring is painful.
bool AP_Generator_GX_7::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_reading_ms > 2000) { // we expect @1Hz
        hal.util->snprintf(failmsg, failmsg_len, "no messages in %ums", unsigned(now - last_reading_ms));
        return false;
    }
    if (SRV_Channels::get_channel_for(SRV_Channel::k_generator_control) == nullptr) {
        hal.util->snprintf(failmsg, failmsg_len, "need a servo output channel");
        return false;
    }

    uint32_t errors = extender_error;

    // requiring maintenance isn't something that should stop
    // people flying - they have work to do.  But we definitely
    // complain about it - a lot.
    errors &= ~(1U << uint32_t(ExtenderError::MAINTENANCE_TIME_ERROR));

    if (errors) {

        for (uint8_t i=0; i<32; i++) {
            if (errors & (1U << i)) {
                if (errors < ExtenderError::LAST) {
                    hal.util->snprintf(failmsg, failmsg_len, "error: %s", error_strings[i]);
                } else {
                    hal.util->snprintf(failmsg, failmsg_len, "unknown error: 1U<<%u", i);
                }
            }
        }
        return false;
    }

    if (working_state != WorkingState::RUN) {
        hal.util->snprintf(failmsg, failmsg_len, "not started");
        return false;
    }

    return true;
}

// Update front end with voltage, current, and rpm values
void AP_Generator_GX_7::update_frontend_readings(void)
{
    _voltage = output_voltage;
    _current = output_current;
    _rpm = engine_speed;
    _fuel_remaining = fuel_level;

    update_frontend();
}


// healthy returns true if the generator is not present, or it is
// present, providing telemetry and not indicating an errors.
bool AP_Generator_GX_7::healthy() const
{
    const uint32_t now = AP_HAL::millis();

    if (last_reading_ms == 0 || now - last_reading_ms > 2000) {
        return false;
    }
    if (extender_error) {
        return false;
    }
    return true;
}


// TODO update this function
// send mavlink generator status
void AP_Generator_GX_7::send_generator_status(const GCS_MAVLINK &channel)
{
    if (last_reading_ms == 0) {
        // nothing to report
        return;
    }

    uint64_t status = 0;
    if (last_reading.rpm == 0) {
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
    } else {
        switch (last_reading.mode) {
        case Mode::OFF:
            status |= MAV_GENERATOR_STATUS_FLAG_OFF;
            break;
        case Mode::IDLE:
            if (pilot_desired_runstate == RunState::RUN) {
                status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
            } else {
                status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
            }
            break;
        case Mode::RUN:
            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
            break;
        case Mode::CHARGE:
            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
            status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
            break;
        case Mode::BALANCE:
            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
            status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
            break;
        }
    }

    if (last_reading.errors & (uint8_t)Errors::Overload) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT;
    }
    if (last_reading.errors & (uint8_t)Errors::LowVoltageOutput) {
        status |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
    }

    if (last_reading.errors & (uint8_t)Errors::MaintenanceRequired) {
        status |= MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED;
    }
    if (last_reading.errors & (uint8_t)Errors::StartDisabled) {
        status |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }
    if (last_reading.errors & (uint8_t)Errors::LowBatteryVoltage) {
        status |= MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
        last_reading.rpm, // generator_speed
        std::numeric_limits<double>::quiet_NaN(), // battery_current; current into/out of battery
        last_reading.output_current, // load_current; Current going to UAV
        std::numeric_limits<double>::quiet_NaN(), // power_generated; the power being generated
        last_reading.output_voltage, // bus_voltage; Voltage of the bus seen at the generator
        INT16_MAX, // rectifier_temperature
        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
        INT16_MAX, // generator temperature
        last_reading.runtime,
        (int32_t)last_reading.seconds_until_maintenance
        );
}

// methods to control the generator state:
bool AP_Generator_GX_7::stop() 
{
    set_pilot_desired_runstate(RunState::STOP); 
    return true;
}

bool AP_Generator_GX_7::start() 
{
    set_pilot_desired_runstate(RunState::START); 
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
