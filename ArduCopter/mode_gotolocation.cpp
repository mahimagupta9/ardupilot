#include "Copter.h"

#if MODE_GOTOLOCATION_ENABLED

bool Mode_GotoLocation::init(bool ignore_checks)
{
    takeoff_complete = false;
    mission_completed = false;
    landing_init_confirm = false;

    flt_plan = FLIGHT_PLAN::STAND_BY;
    
    pos_control_start();
    
    return true;
}

// initialise position controller
void Mode_GotoLocation::pos_control_start()
{        
    hal.console->printf("Initialising go-to mode\n");
    
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
}

// should be called at 100hz or more
void Mode_GotoLocation::run()
{
    switch (flt_plan)
    {
        case Mode_GotoLocation::FLIGHT_PLAN::STAND_BY:
            if (mission_completed)
            {
                // hal.console->printf("mission completed\n");
                return;
            }
            else if(copter.ap.pre_arm_check)
            {
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
                mode_goto_loc_takeoff();
            }
        break;

        case Mode_GotoLocation::FLIGHT_PLAN::TAKEOFF:
            takeoff_run();
        break;        
        
        case Mode_GotoLocation::FLIGHT_PLAN::GO_TO_WP:
            // hal.console->printf("In go_to_wp\n");
        break;        
        
        case Mode_GotoLocation::FLIGHT_PLAN::LAND_AND_DISARM:
            if (!landing_init_confirm)
            {
                hal.console->printf("Landing start\n");
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

bool Mode_GotoLocation::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        hal.console->printf("Already armed\n");
        return true;
    }

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);
    return true;
}

void Mode_GotoLocation::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        hal.console->printf("Already disarmed\n");
        return;
    }

    // arm
    motors->armed(false);
    hal.util->set_soft_armed(false);
    return;
}

void Mode_GotoLocation::mode_goto_loc_takeoff()
{
    // hal.console->printf("mode_goto_loc_takeoff\n");
    float float_takeoff_cm = 21 * 100.0f;    
    bool confirm_takeoff_start = copter.flightmode->do_user_takeoff(float_takeoff_cm,0);
    if (confirm_takeoff_start)
    {
        hal.console->printf("Takeoff start\n");
        flt_plan = Mode_GotoLocation::FLIGHT_PLAN::TAKEOFF;    
    }   
}

bool Mode_GotoLocation::do_user_takeoff_start(float takeoff_alt_cm)
{
    // hal.console->printf("In do_user_takeoff_start\n");
    
    int32_t alt_target_cm = takeoff_alt_cm; 
    
    Location target_loc = copter.current_loc;
    target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);
    
    // provide target altitude as alt-above-ekf-origin
    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
        hal.console->printf("cannot provide alt above ekf-origin\n");
        return false;
    }

    pos_control->init_z_controller();
    auto_takeoff.start(alt_target_cm, false);

    takeoff_complete = false;
    return true;
}

void Mode_GotoLocation::takeoff_run()
{
    // hal.console->printf("In takeoff_run\n");
    auto_takeoff.run();
    if (auto_takeoff.complete && !takeoff_complete) {
        hal.console->printf("Takeoff complete\n");
        takeoff_complete = true;
        // flt_plan = Mode_GotoLocation::FLIGHT_PLAN::GO_TO_WP;
        
    //for testing
        flt_plan = Mode_GotoLocation::FLIGHT_PLAN::LAND_AND_DISARM; 
    //for testing
    }    
}

//To-do: call init_landing() when go-to-wp case is finished and start_landing() in LnD case 
bool Mode_GotoLocation::init_landing()
{
    // hal.console->printf("init_landing\n");
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
    
    start_landing();    
    return true;
}

void Mode_GotoLocation::start_landing()
{
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        hal.console->printf("land detected\n");
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
