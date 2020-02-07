/*
** Texas A&M University
** Team: Crane Tech
** Author: Luis Orozco
** File: overlay.py
** Description: 
   File responsible for holding functions necessary for overlaying textual information
   on video frames taken by the camera and saving videos onboard the raspberry pi
 */


#include "Copter.h"


/* 
** Name: init

** Description: when called will check to see if it is okay for vehicle to enter new mode.
    If it is okay to enter mode, the init function will return true

** Parameters:
    - ignore_checks: boolean values to determine whether or not to ignore checks

** Return:
    - value signifying whether or not it is okay to enter mode

** Notes:
 */
bool ModeBIRD::init(bool ignore_checks)
{
    // if (!ignore_checks) {
        // if (!AP::ahrs().home_is_set()) {
        //     return false;
        // }
    // }

    // initialise waypoint and spline controller
    // wp_nav->wp_and_spline_init();
    // _state = RTL_Starting;
    // _state_complete = true; // see run() method below
    // terrain_following_allowed = !copter.failsafe.terrain;

    return true;
}


/*
** Name: run

** Description: recevies commands from transmitter, pilot's inputs, and converts
    them to motor values

** Parameters: none

** Return: none

** Notes:
    stabilize_run - runs the main stabilize controller
    should be called at 100hz or more
 */
void ModeBIRD::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) 
    {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } 

    else if (copter.ap.throttle_zero) 
    {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } 

    else 
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) 
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) 
        {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
}
