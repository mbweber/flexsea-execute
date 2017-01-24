/****************************************************************************
    [Project] FlexSEA: Flexible & Scalable Electronics Architecture
    [Sub-project] 'flexsea-user' User projects
    Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

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
*****************************************************************************
    [Lead developper] Luke Mooney, lmooney at dephy dot com.
    [Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
    Biomechatronics research group <http://biomech.media.mit.edu/>
    [Contributors]
*****************************************************************************
    [This file] flexsea_pid_controller.h : generic pid controller object interface
*****************************************************************************
    [Change log] (Convention: YYYY-MM-DD | author | comment)
    * 2017-01-10 | dweisdorf | New release
****************************************************************************/

#ifndef INC_FLEXSEA_PID_CONTROLLER_H
#define INC_FLEXSEA_PID_CONTROLLER_H   

#include "stdint.h"

/*  This interface defines a generic closed loop PID controller that can be used for anything
    
    General/Example usage:
    
        //declare pid_controller myController
        pid_controller_initialize(&myController, MY_MAX_ERROR_SUM);
        pid_controller_setGains(&myController, KP, KI, KD, resolution);
        //optionally turn off critical failure stuff
        pid_controller_settings(&myController, 0, 1);
    
            ...
    
        //in a finiteStateMachine:
        myController.actualValue = getValueFromSensor(...);
        // optionally update set point and gains
        myController.setpoint = someValue;
        pid_controller_setGains(...);
        
        pwm = pid_controller_compute(&myController);
        sign = pwm > 0 ? 1 : -1;
        motor_open_speed_2(pwm * sign, sign);
 *
*/
    
typedef struct pid_controller_s {
    
    //Gains:
    int32_t KP;
    int32_t KI;
    int32_t KD;
    
    int32_t controlValue;
    int32_t setpoint;

    //Errors:
    int32_t error;
    int32_t errorSum;                   //sum is proportional to integral
    int32_t errorDifference;
    int32_t errorPrevious;              //Error at previous call to controller
    
    //Critical Fault Prevention:
    int32_t outputMax;
    int32_t controlValueMax;
    int32_t controlValueLimitingThreshold;
    int32_t errorSumMax; 
    
    //Other:
    uint8_t firstTime;
    uint8_t settings;                   //A bitmap, each bit represents a setting. Check #defines at bottom to see which bit represents what
    
    /*  resolutionFactor should be set between ? - ? ( 6-10 for now)
        ToDo: some math to figure out reasonable limits
    
        Resolution factor will set resolution to 2^(resolution factor)
        BUT IT WILL ALSO decrease magnitude of gains by same factor
        thus it is also a sort of scale factor. If inputs are very large, a larger resolution factor may be appropriate
    
        ie: resolutionFactor = 0 w/ Gains 8, 8, 4
            is equal to 
            resolutionFactor = 1 w/ Gains 16, 16, 8 
            is equal to 
            resolutionFactor = 2 w/ Gains 32, 32, 16, and so on 
            
            however considering 
            resolutionFactor = 2 w/ Gains 33, 35, 13
            there is no equivalent set of gains for resolutionFactor = 1, since gains are ints
    */
    uint8_t resolutionFactor;           
    
} pid_controller;
    
void pid_controller_initialize(pid_controller* controller, int32_t outputMax, int32_t controlValueMax, uint8_t controlValueLimitPercent, int32_t errorSumMax);
void pid_controller_settings(pid_controller* controller, short useControlValueMax, short useErrorMax, short useOutputMax);
void pid_controller_setGains(pid_controller* controller, int32_t KP, int32_t KI, int32_t KD,  uint8_t resolutionFactor);
int32_t pid_controller_compute(pid_controller* ctrl);
int32_t pid_controller_compute_ff(pid_controller* ctrl, int32_t feedForward);

int32_t pid_controller_predict(pid_controller* ctrl);


// Settings
#define USE_CONTROL_VALUE_MAX 0x02
#define USE_ERROR_SUM_MAX 0x04
#define USE_OUTPUT_MAX 0x08

#endif    
