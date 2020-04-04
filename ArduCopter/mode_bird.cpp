/*
** Texas A&M University
** Team: Crane Tech
** Author: Luis Orozco
** File: overlay.py
** Description: 
   Flight mode responsible for simulating flight. A user must carry the drone and simulate
   the flight path. Used to test the logic and flow of software without outputting 
   power to the motors. Messages are sent to the ground station to show which state the 
   drone is in. 
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

    gcs().send_text(MAV_SEVERITY_INFO, "State 0 = Ascend");
    gcs().send_text(MAV_SEVERITY_INFO, "State 1 = Moving Right");
    gcs().send_text(MAV_SEVERITY_INFO, "State 2 = Descend");
    gcs().send_text(MAV_SEVERITY_INFO, "State 3 = Moving Right");


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

    // capture current lidar reading
    busyFlag = sensor.garminGetBusyFlag();

    if (busyFlag == 0x00)
    {
        current_altitude = sensor.garminReadDistance();
        sensor.garminTakeRange();
    }

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

            // reading is beyond the expected range
            if(reading > 400)
            {
                // has the drone moved more than 1.5 meters and 2 meters
                if(delta_altitude >= 150 && delta_altitude <= 200)
                {
                    // wall has ended and it is time to land
                    copter.set_mode(Mode::Number::LAND, ModeReason::UNKNOWN);
                    break;
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
                // move forward
                bird_run_move_forward(counter);
            }

            // does the drone need to move backward to correct itself
            if(reading <= 100)
            {
                // move backward
                bird_run_move_backward(counter);
            }

            break;

        // state 1: move right
        case 1:
            // count value is hypothetical
            // fruther testing require to find value to move 1.4 m to the right
            if(count < 800)
            {
                // move right
                count++;
            }
            
            // increment state
            else
            {
                state++;
                count = 0;
                previous_altitude = current_altitude;
                break;
            }

            break;

        // state 2: descend
        case 2:
            // descend

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
                    break;
                }

            }

            // does the drone need to move forward to correct itself 
            if(reading =< 400 && reading >= 200)
            {
                // move forward
                bird_run_move_forward(counter);
            }

            // does the drone need to move backward to correct itself
            if(reading <= 100)
            {

                // move backward
                bird_run_move_backward(counter);
            }

            break;

        // state 3: move right
        case 3:
            // count value is hypothetical
            // fruther testing require to find value to move 1.4 m to the right
            if(count < 500)
            {                
                // move right
                count++;
                break;
            }
            
            // reset state
            else
            {
                state = 0;
                count = 0;
                previous_altitude = current_altitude;
                break;
            }

            break;

        // default
        default:
            state = 0;
            count = 0;
            break;

    }

    // for debugging; will send a message to ground station for user to see state
    if(counter >= 400)
    {
        //
        counter = 0;

        gcs().send_text(MAV_SEVERITY_INFO, "^^^ State ^^^");
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
** Name: bird_run_move_forward

** Description: move the drone forward by changing the pitch of the drone

** Parameters:
    - target_roll: desired value for roll
    - target_pitch: desired value for pitch
    - target_yaw_rate: desured value for yaw

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_move_forward(int counter)
{
    if(counter >= 400)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Drone too far from wall");
        gcs().send_text(MAV_SEVERITY_INFO, "Moving Forward");
    }
}


/* 
** Name: bird_run_move_backward

** Description: will move the drone backward by changing the pitch of the drone

** Parameters:
    - target_roll: desired value for roll
    - target_pitch: desired value for pitch
    - target_yaw_rate: desured value for yaw

** Return:
    N/A

** Notes:
 */
void ModeBIRD::bird_run_move_backward(int counter)
{
    //
    if(counter >= 400)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Drone too close from wall");
        gcs().send_text(MAV_SEVERITY_INFO, "Moving Backward");
    }
}


