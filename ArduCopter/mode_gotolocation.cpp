#include "Copter.h"

#if MODE_GOTOLOCATION_ENABLED

#define TAKEOFF_ALT_CM  20 
#define NEW_LOCATION_HORZ_DISTANCE_M    100.0f

bool Mode_GotoLocation::init(bool ignore_checks)
{
    takeoff_complete = false;
    landing_init_confirm = false;
    
    pos_control_start();
    
    return true;
}

// initialise position controller
void Mode_GotoLocation::pos_control_start()
{            
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt    
    gotoloc_pos_terrain_alt = false;
}

// should be called at 100hz or more
void Mode_GotoLocation::run()
{
    switch (flt_plan)
    {
        case Mode_GotoLocation::FLIGHT_PLAN::STAND_BY:
            if (mission_completed)
            {
                // gcs().send_text(MAV_SEVERITY_INFO, "mission completed");
                return;
            }
            else 
            {
                if(copter.ap.pre_arm_check)
                    flt_plan = Mode_GotoLocation::FLIGHT_PLAN::ARM;
            }
            break;   

        case Mode_GotoLocation::FLIGHT_PLAN::ARM:
            if (!motors->armed())
            {
                arm_motors();
            }
            else
            {
                if(!takeoff_complete)
                {
                    if(mode_goto_loc_takeoff())
                        flt_plan = FLIGHT_PLAN::TAKEOFF;
                }    
                else
                {
                    flt_plan = Mode_GotoLocation::FLIGHT_PLAN::GO_TO_WP;
                }                          
            }
        break;

        case Mode_GotoLocation::FLIGHT_PLAN::TAKEOFF:
            if (!takeoff_complete)
            {
                takeoff_run();
            }
            else
            {
                flt_plan = Mode_GotoLocation::FLIGHT_PLAN::GO_TO_WP;
            }
        break;        
        
        case Mode_GotoLocation::FLIGHT_PLAN::GO_TO_WP:
            if (!wp_reached_init)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Moving towards new location");
                set_new_location();
            }
            else
            {
                location_run();
                // check if we've reached the location
                if (wp_distance() < 10) 
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "location reached");
                    flt_plan = Mode_GotoLocation::FLIGHT_PLAN::LAND_AND_DISARM;
                }
            }                        
        break;        
        
        case Mode_GotoLocation::FLIGHT_PLAN::LAND_AND_DISARM:
            if (!landing_init_confirm)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Landing");
                init_landing();
            } 
            else 
            {
                if (motors->armed())
                {
                    start_landing();
                }
                else
                {
                    mission_completed = true;
                    flt_plan = Mode_GotoLocation::FLIGHT_PLAN::STAND_BY;
                }
            }                
        break;

    default:
        break;
    }
}

void Mode_GotoLocation::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already armed");
        return;
    }

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);
    return;
}

void Mode_GotoLocation::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already disarmed");
        return;
    }

    // arm
    motors->armed(false);
    hal.util->set_soft_armed(false);
    return;
}

bool Mode_GotoLocation::mode_goto_loc_takeoff()
{
    float float_takeoff_cm = TAKEOFF_ALT_CM * 100.0f;    
    bool confirm_takeoff_start = copter.flightmode->do_user_takeoff(float_takeoff_cm,0);
    if (confirm_takeoff_start)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff start to %d m",(uint8_t)(float_takeoff_cm/100)); 
    }   
    return confirm_takeoff_start;
}

bool Mode_GotoLocation::do_user_takeoff_start(float takeoff_alt_cm)
{    
    int32_t alt_target_cm = takeoff_alt_cm; 
    
    Location target_loc = copter.current_loc;
    target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);
    
    // provide target altitude as alt-above-ekf-origin
    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Cannot provide alt above ekf-origin");
        return false;
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();

    // initialise alt 
    auto_takeoff.start(alt_target_cm, false);

    // record takeoff has not completed
    takeoff_complete = false;
    return true;
}

void Mode_GotoLocation::takeoff_run()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "In takeoff_run");
    auto_takeoff.run();
    if (auto_takeoff.complete && !takeoff_complete) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete");
        takeoff_complete = true;
    }    
}

void Mode_GotoLocation::set_new_location()
{
    // prepare position
    Vector3f pos_vector;
    float x = NEW_LOCATION_HORZ_DISTANCE_M, y = 0.0f, z = 0.0f;
    // convert to cm
    pos_vector = Vector3f(x * 100.0f, y * 100.0f, -z * 100.0f);
    // rotate to body-frame
    copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
    // add body offset
    pos_vector += copter.inertial_nav.get_position_neu_cm();

    auto_yaw.set_mode_to_default(false);

    pos_control_start();

    pos_control->set_pos_offset_z_cm(0.0);

    // set position target and zero velocity and acceleration
    gotoloc_pos_target_cm = pos_vector.topostype();

    wp_reached_init = true;
}

bool Mode_GotoLocation::get_wp(Location& destination) const
{
    // to avoid getting initial location as (0,0,0)
    float x = gotoloc_pos_target_cm.x;
    float y = gotoloc_pos_target_cm.y;
    float z = gotoloc_pos_target_cm.y;
    if (x <= 0.0f && y <= 0.0f && z <= 0.0f)
    {
        return false;
    }
    
    destination = Location(gotoloc_pos_target_cm.tofloat(), gotoloc_pos_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    return true;    
}

void Mode_GotoLocation::location_run()
{
     // if not armed set throttle to zero and exit immediately
     if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller    
    float terr_offset = 0.0f;
    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    pos_control->input_pos_xyz(gotoloc_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

uint32_t Mode_GotoLocation::wp_distance() const
{
    return get_horizontal_distance_cm(inertial_nav.get_position_xy_cm(), gotoloc_pos_target_cm.tofloat().xy());   
}

bool Mode_GotoLocation::init_landing()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "init_landing");
    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    landing_init_confirm = true;    
//starts landing sequence    
    start_landing();    
    
    return true;
}

void Mode_GotoLocation::start_landing()
{
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        gcs().send_text(MAV_SEVERITY_INFO, "land detected");
        disarm_motors();
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        land_run_horiz_and_vert_control();
    }
}

#endif
