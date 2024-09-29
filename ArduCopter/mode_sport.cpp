#include "Copter.h"

#if MODE_SPORT_ENABLED == ENABLED

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    _cancelled_landing = false;
    _landing = false;

    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, sport_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        sport_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        sport_nav->clear_pilot_desired_acceleration();
    }
    sport_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if AC_PRECLAND_ENABLED
    _precision_sport_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeSport::do_precision_sport()
{
    if (!_precision_sport_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (sport_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeSport::precision_sport_xy()
{
    sport_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        sport_nav->clear_pilot_desired_acceleration();
    }

    // relax sport target if we might be landed
    if (copter.ap.land_complete_maybe) {
        sport_nav->soften_for_landing();
    }

    // Sport State Machine Determination
    AltHoldModeState sport_state = get_alt_hold_state(target_climb_rate);

    // Sport State Machine
    switch (sport_state) {

    case AltHold_MotorStopped:
        _landing = false;
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        sport_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(sport_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Landed_Ground_Idle:
        _landing = false;
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        _landing = false;
        attitude_control->reset_rate_controller_I_terms_smoothly();
        sport_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(sport_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        _landing = false;
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }


        // get avoidance adjusted climb rate
        if (target_climb_rate >= 0) {
            // get takeoff speed from parameter
            target_climb_rate = g.pilot_takeoff_spd;
            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        } else {
            target_climb_rate = 0;
        }

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run sport controller
        sport_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(sport_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        bool rng_alt_ok = copter.rangefinder_alt_ok();

        if (!rng_alt_ok) {
            copter.set_mode(Mode::Number::RTL, ModeReason::BAD_DEPTH);
        }

        Matrix3f rngRotMatrix = {
                Vector3f{copter.ahrs.cos_pitch(),   copter.ahrs.sin_pitch() * copter.ahrs.sin_roll(),   copter.ahrs.sin_pitch() * copter.ahrs.cos_roll()}, 
                Vector3f{0,                         copter.ahrs.cos_roll(),                             -copter.ahrs.sin_roll()},
                Vector3f{-copter.ahrs.sin_pitch(),  copter.ahrs.cos_pitch() * copter.ahrs.sin_roll(),   copter.ahrs.cos_pitch() * copter.ahrs.cos_roll()}
            };

            int16_t fc_height_rng = copter.rangefinder_state.alt_cm_glitch_protected - (rngRotMatrix * copter.rangefinder.get_pos_offset_orient(ROTATION_PITCH_270)).z;

            if (
                !_cancelled_landing && (_landing || (
                    fc_height_rng <= (g.pilot_takeoff_alt + (copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270)) + 10) && 
                    target_climb_rate <= -get_pilot_speed_dn()*0.99 && 
                    copter.gps.ground_speed_cm() <= 50
                ))
            ) {

                bool cancel_landing = false;

                if (!_landing) {
                    _land_start_time = millis();
                    _land_pause = true;
                    _landing = true;
                    _message_sent = false;
                }

                // pause before beginning land descent
                if (_land_pause && millis()-_land_start_time >= (uint16_t) g.pilot_land_delay) {
                    _land_pause = false;
                }

                // cancel landing if throttle is not at minimum and the landing descent has not yet started
                if (
                    _land_pause && (
                        target_climb_rate > -get_pilot_speed_dn()*0.99 || 
                        channel_yaw->norm_input_dz() > 0.1 || 
                        channel_yaw->norm_input_dz() < -0.1 || 
                        channel_pitch->norm_input_dz() > 0.1 || 
                        channel_pitch->norm_input_dz() < -0.1 || 
                        channel_roll->norm_input_dz() > 0.1 || 
                        channel_roll->norm_input_dz() < -0.1
                        )
                    ) {
                    cancel_landing = true;
                }

                // cancel landing if landing descent has started and high throttle cancels landing is set and throttle is high
                if (!_land_pause && (g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND_LOITER_SPORT) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
                    cancel_landing = true;
                }

                if (_landing && _land_pause) {
                    // convert pilot input to lean angles
                    get_pilot_desired_lean_angles(target_roll, target_pitch, sport_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

                    // process pilot's roll and pitch input
                    sport_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

#if AC_PRECLAND_ENABLED
                            bool precision_sport_old_state = _precision_sport_active;
                    if (do_precision_sport()) {
                        precision_sport_xy();
                        _precision_sport_active = true;
                    } else {
                        _precision_sport_active = false;
                    }
                    if (precision_sport_old_state && !_precision_sport_active) {
                        // prec sport was active, not any more, let's init again as user takes control
                        sport_nav->init_target();
                    }
                    // run sport controller if we are not doing prec sport
                    if (!_precision_sport_active) {
                        sport_nav->update();
                    }
#else
                    sport_nav->update();
#endif
                }

                if (_landing) {
                    if (!_land_pause && !_message_sent && !cancel_landing){
                        gcs().send_text(MAV_SEVERITY_ALERT,"Landing Initiated");
                        _message_sent = true;
                    }
                    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
                    land_run_horizontal_control();
                    land_run_vertical_control(_land_pause);
                    if (cancel_landing) {
                        _landing = false;
                        if (!_land_pause) {
                            _cancelled_landing = true;
                            if (_message_sent){
                                gcs().send_text(MAV_SEVERITY_ALERT,"Landing Cancelled");
                            }
                        }
                    }
                }

            }
            else 
            {
                if (_cancelled_landing){
                    if (fc_height_rng < (g.pilot_takeoff_alt + (copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270))) + 10) {
                        sport_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
#if AC_PRECLAND_ENABLED
                        bool precision_sport_old_state = _precision_sport_active;
                        if (do_precision_sport()) {
                            precision_sport_xy();
                            _precision_sport_active = true;
                        } else {
                            _precision_sport_active = false;
                        }
                        if (precision_sport_old_state && !_precision_sport_active) {
                            // prec sport was active, not any more, let's init again as user takes control
                            sport_nav->init_target();
                        }
                        // run sport controller if we are not doing prec sport
                        if (!_precision_sport_active) {
                            sport_nav->update();
                        }
#else
                        sport_nav->update();
#endif

                        attitude_control->input_thrust_vector_rate_heading(sport_nav->get_thrust_vector(), 0.0f);

                        target_climb_rate = MAX(target_climb_rate, MIN(50, g.pilot_speed_up));

                        // get avoidance adjusted climb rate
                        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

                        // update the vertical offset based on the surface measurement
                        copter.surface_tracking.update_surface_offset();

                        // Send the commanded climb rate to the position controller
                        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
                        break;
                    }
                    else
                    {
                        _cancelled_landing = false;
                    }
                }

                if (copter.rangefinder_state.alt_cm_glitch_protected < copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270))
                {
                    copter.set_mode(Mode::Number::RTL, ModeReason::BAD_DEPTH);
                    gcs().send_text(MAV_SEVERITY_ALERT,"Rangefinder Occluded: Returning to Home");
                }

                // convert pilot input to lean angles
                get_pilot_desired_lean_angles(target_roll, target_pitch, sport_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

                // process pilot's roll and pitch input
                sport_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

#if AC_PRECLAND_ENABLED
                bool precision_sport_old_state = _precision_sport_active;
                if (do_precision_sport()) {
                    precision_sport_xy();
                    _precision_sport_active = true;
                } else {
                    _precision_sport_active = false;
                }
                if (precision_sport_old_state && !_precision_sport_active) {
                    // prec sport was active, not any more, let's init again as user takes control
                    sport_nav->init_target();
                }
                // run sport controller if we are not doing prec sport
                if (!_precision_sport_active) {
                    sport_nav->update();
                }
#else
                sport_nav->update();
#endif

                // call attitude controller
                attitude_control->input_thrust_vector_rate_heading(sport_nav->get_thrust_vector(), target_yaw_rate);

                if (fc_height_rng < (g.pilot_takeoff_alt + (copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270))) - 10) {
                    target_climb_rate = MAX(target_climb_rate, MIN(powF(fc_height_rng - g.pilot_takeoff_alt, 2) / (powF(g.pilot_takeoff_alt, 2)), 1) * g.pilot_speed_up);
                    // target_climb_rate = MAX(target_climb_rate, MIN((g.pilot_takeoff_alt - fc_height_rng) / (g.pilot_takeoff_alt), 1) * g.pilot_speed_up);

                }
                else if (target_climb_rate < 0) {
                    if (g.pilot_takeoff_alt > 0.0f){
                        target_climb_rate = MAX(target_climb_rate, MIN(powF((fc_height_rng - (g.pilot_takeoff_alt - 10)) / (2 * get_pilot_speed_dn()), 2) + 0.05, 1) * -get_pilot_speed_dn());
                        // target_climb_rate_mmps = MAX(target_climb_rate_mmps, MIN((alt_above_ground_mm - takeoff_landing_alt_mm) / (takeoff_landing_alt_mm / 1000), 1000) * -get_pilot_speed_dn());

                        if (fc_height_rng <= g.pilot_takeoff_alt + (copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270))) {
                            target_climb_rate = 0;
                        }
                    }
                }

                // get avoidance adjusted climb rate
                target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

                // update the vertical offset based on the surface measurement
                copter.surface_tracking.update_surface_offset();

                // Send the commanded climb rate to the position controller
                pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            }
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeSport::wp_distance() const
{
    return sport_nav->get_distance_to_target();
}

int32_t ModeSport::wp_bearing() const
{
    return sport_nav->get_bearing_to_target();
}

bool ModeSport::is_landing() const
{
    return _landing;
}

void ModeSport::exit()
{
    _landing = false;
}

#endif
