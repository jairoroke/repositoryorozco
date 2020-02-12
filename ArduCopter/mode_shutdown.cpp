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
bool ModeShutdown::init(bool ignore_checks)
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
void ModeShutdown::run()
{

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);

    // output pilot's throttle
    attitude_control->set_throttle_out(0,
                                       true,
                                       g.throttle_filt);
}
