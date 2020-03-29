/*
** Texas A&M University
** Team: Crane Tech
** Author: Luis Orozco
** File: overlay.py
** Description: 
   Flight mode responsible for maintaining a certain distance from the wall and 
   following the following predefine flight path: up, right, down, right, 
   continously until the wall ends. 
 */


#include "Copter.h"
#include "linux/types.h"
#include "cstdio"
#include <AP_RangeFinder/AP_RangeFinder_I2C_Sensors.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>

static int count = 0;
static int counter = 0;
static int counter2 = 0;
static int state = 0;
static __u16 previous_altitude = 0;
static __u16 current_altitude = 0;
static __u16 delta_altitude = 0;
static __u8  busyFlag = 0;

static __u16 reading = 150;
char messageSent[20];

I2CSensors sensor;


/* 
** Name: init

** Description: when called will check to see if it is okay for vehicle to enter new mode.
    If it is okay to enter mode, the init function will return true

** Parameters:
    - ignore_checks: boolean values to determine whether or not to ignore checks

** Return:

** Notes:
 */
bool ModeBIRD::init(bool ignore_checks)
{
    
    sensor.i2c_init();
    sensor.sonarTakeRange();
    sensor.garmin_configure(0);
    sensor.garminTakeRange();

    // should be a lidar reading
    busyFlag = sensor.garminGetBusyFlag();

    if (busyFlag == 0x00)
    {
        previous_altitude = sensor.garminReadDistance();
        sensor.garminTakeRange();
    }

    auto_yaw.set_mode(AUTO_YAW_HOLD);


    return true;
}


/*
** Name: run

** Description: will follow predefined path, up, right, down, right, until it 
    completes a section of the wall. 

** Parameters: none

** Return: none

** Notes:
    should be called at 400 Hz becasue of this mode needs to be fast and efficient
 */
void ModeBIRD::run()
{
    
    // define roll, pitch, and yaw
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;

    // capture current altitude from lidar reading
    current_altitude = sensor.garminReadDistance();
    sensor.garminTakeRange();

    // determine change in altitude from state to state
    // such as, how much has the drone moved in state 0
    delta_altitude = abs(previous_altitude - current_altitude);
    
    // capture sonar reading when counter expires
    if(counter2 > 50)
    {
        //
        reading = sensor.sonarGetRangeRead();
        sensor.sonarTakeRange();
        counter2 = 0;
    }

    else
    {
        //
        counter2 += 1;
    }

    // carry out the function of the specific state
    switch(state)
    {
        // state 0: Ascend
        case 0:
            // ascend
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            bird_run_ascend_control();

            // reading is beyond the expected range
            if(reading > 400)
            {
                // has the drone moved more than 1.5 meters and 2 meters
                if(delta_altitude >= 150 && delta_altitude <= 200)
                {
                    // wall has ended and it is time to land
                    copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
                }

                // if the drone has moved more than 2 meters
                if(delta_altitude > 200)
                {
                    // it is time to switch to the next state
                    state++;
                    previous_altitude = current_altitude;
                    break;
                }

            }

            // does the drone need to move forward to correct itself
            if(reading =< 400 && reading >= 200)
            {
                bird_run_move_forward(0, -1000, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
            }

            // does the drone need to move backward to correct itself
            if(reading <= 100)
            {
                bird_run_move_backward(0, 1000, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
            }

            break;

        // state 1: move right
        case 1:
            // count value is hypothetical
            // fruther testing require to find value to move 1.4 m to the right
            if(count < 500)
            {
                bird_run_move_right(1000, 0, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
                count++;
            }
            
            // reset state
            else
            {
                state++;
                count = 0;
            }

            break;

        // state 2: descend
        case 2:
            // descend
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            bird_run_descend_control();

            // is the drone 1 meter or less above the ground
            if(current_altitude <= 100)
            {   
                // time to move on to the next state
                state++;
                previous_altitude = current_altitude;
                break;
            }

            // reading is beyond the expected range
            if(reading > 400)
            {
                // has the drone moved more than 1.5 meters 
                if(delta_altitude >= 150)
                {
                    // there is no wall as the drone is descending
                    // wall has ended and time to land
                    copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
                }

            }

            // does the drone need to move forward to correct itself 
            if(reading =< 400 && reading >= 200)
            {
                bird_run_move_forward(0, -1000, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
            }

            // does the drone need to move backward to correct itself
            if(reading <= 100)
            {

                bird_run_move_backward(0, 1000, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
            }

            break;

        // state 3: move right
        case 3:
            // count value is hypothetical
            // fruther testing require to find value to move 1.4 m to the right
            if(count < 500)
            {                
                bird_run_move_right(1000, 0, 0); // 1:target_roll , 2:target_pitch , 3:target_yaw_rate
                count++;
            }
            
            // reset state
            else
            {
                state = 0;
                count = 0;
            }

            break;

        // default
        default:
            state = 0;
            count = 0;
            break;

    }

    // for debugging; will send a message to ground station for user to see state
    if(counter >= 800)
    {
        //
        counter = 0;

        gcs().send_text(MAV_SEVERITY_INFO, "State: ");
        snprintf(messageSent, sizeof(messageSent), "%d", state);
        gcs().send_text(MAV_SEVERITY_INFO, messageSent);

    }

    else
    {
        //
        counter += 1;
    }

}


/* 
** Name: bird_run_ascend_control

** Description: it is a copy of land mode function called descend control,
    has been modified such that it uses the max speed up so it ascends
    rather than descend

** Parameters:
    N/A

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_ascend_control()
{
    //
    float cmb_rate = 0;
    float max_land_ascend_velocity;
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    max_land_ascend_velocity = pos_control->get_max_speed_up();

    // Don't speed up for ascension
    max_land_ascend_velocity = MIN(max_land_ascend_velocity, abs(g.land_speed));

    // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
    cmb_rate = AC_AttitudeControl::sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z(), G_Dt);

    // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
    cmb_rate = constrain_float(cmb_rate, max_land_ascend_velocity, abs(g.land_speed));


    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control->update_z_controller();
}


/* 
** Name: bird_run_descend_control

** Description: it is a copy of land mode function called descend control

** Parameters:
    N/A

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_descend_control()
{
    float cmb_rate = 0;
    float max_land_descent_velocity;
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    max_land_descent_velocity = pos_control->get_max_speed_down();

    // Don't speed up for landing.
    max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

    // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
    cmb_rate = AC_AttitudeControl::sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z(), G_Dt);

    // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
    cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control->update_z_controller();
}


/* 
** Name: bird_run_move_forward

** Description: move the drone forward

** Parameters:
    - target_roll: desired value for roll
    - target_pitch: desired value for pitch
    - target_yaw_rate: desured value for yaw

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_move_forward(float target_roll, float target_pitch, float target_yaw_rate)
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot's desired yaw rate
    if (!is_zero(target_yaw_rate)) 
    {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }


    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(0, true, g.throttle_filt);
}


/* 
** Name: bird_run_move_backward

** Description: will move the drone backward

** Parameters:
    - target_roll: desired value for roll
    - target_pitch: desired value for pitch
    - target_yaw_rate: desured value for yaw

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_move_backward(float target_roll, float target_pitch, float target_yaw_rate)
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot's desired yaw rate
    if (!is_zero(target_yaw_rate)) 
    {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }


    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(0, true, g.throttle_filt);   
}


/* 
** Name: bird_run_move_right

** Description: will move the drone to the right 

** Parameters:
    - target_roll: desired roll
    - target_pitch: desired pitch
    - target_yaw_rate: desired yaw

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_move_right(float target_roll, float target_pitch, float target_yaw_rate)
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot's desired yaw rate
    if (!is_zero(target_yaw_rate)) 
    {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(0, true, g.throttle_filt);   
}

