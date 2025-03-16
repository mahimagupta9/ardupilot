#include "Copter.h"

#if MODE_GOTOLOCATION_ENABLED

bool Mode_GotoLocation::init(bool ignore_checks)
{
    return true;
}

// should be called at 100hz or more
void Mode_GotoLocation::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
}

void Mode_GotoLocation::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);
    // // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
}

void Mode_GotoLocation::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // disarm
    motors->armed(false);
    hal.util->set_soft_armed(false);
}

void Copter::mode_goto_loc_flight_plan()
{
    if (copter.ap.pre_arm_check)
    {
        mode_go_to_location.arm_motors();
    }
    
// takeoff
    float float_takeoff_cm = 10*100;    //10 m
    if (motors->armed())
    {
        // do takeoff through Guided mode or use its controller or create a pos controller for this mode
        copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND);
        // hal.console->printf("armed, taking off\n");
        copter.flightmode->do_user_takeoff(float_takeoff_cm,0);
    }
    // disarm_motors();
}

#endif
