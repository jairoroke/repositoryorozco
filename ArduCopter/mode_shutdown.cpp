/*
** Texas A&M University
** Team: Crane Tech
** Author: Luis Orozco
** File: mode_shutdown.cpp
** Description: 
   Flight mode responsible for stopping the motors.
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
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);

    // output pilot's throttle
    attitude_control->set_throttle_out(0,
                                       false,
                                       g.throttle_filt);

    return true;
}


/* 
** Name: run

** Description: is an exact copy of brake

** Parameters:
    N/A

** Return:
    N/A

** Notes:
 */
void ModeShutdown::run()
{
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
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            
        case AP_Motors::SpoolState::SPOOLING_UP:
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            break;

        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // do nothing
            break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);

    // output pilot's throttle
    attitude_control->set_throttle_out(0,
                                       false,
                                       g.throttle_filt);

}


