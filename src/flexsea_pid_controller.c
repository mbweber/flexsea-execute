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
    [This file] flexsea_pid_controller.c : generic pid controller object implementation
*****************************************************************************
    [Change log] (Convention: YYYY-MM-DD | author | comment)
    * 2017-01-10 | dweisdorf | New release
****************************************************************************/

#include "flexsea_pid_controller.h"

void pid_controller_initialize(pid_controller* controller, int32_t outputMax, int32_t controlValueMax, uint8_t controlValueLimitPercent, int32_t errorSumMax) 
{
    controller->KP = 0;
    controller->KI = 0;
    controller->KD = 0;

    controller->controlValue = 0;
    controller->setpoint = 0;    
    
    controller->error = 0;
    controller->errorDifference = 0;
    controller->errorSum = 0;
    controller->errorPrevious = 0;

    controller->outputMax = outputMax;
    controller->controlValueMax = controlValueMax;
    controller->controlValueLimitingThreshold = controlValueLimitPercent * controlValueMax / 100;
    controller->errorSumMax = errorSumMax;
    
    controller->firstTime = 1;
    
    //by default we will turn these on
    controller->settings = USE_CONTROL_VALUE_MAX | USE_ERROR_SUM_MAX | USE_OUTPUT_MAX;
}

void pid_controller_settings(pid_controller* controller, short useControlValueMax, short useErrorMax, short useOutputMax) 
{
    controller->settings = 0;
    if(useControlValueMax)
    {
        controller->settings |= USE_CONTROL_VALUE_MAX;
    }
    if(useErrorMax) 
    {
        controller->settings |= USE_ERROR_SUM_MAX;
    }
    if(useOutputMax)
    {
        controller->settings |= USE_OUTPUT_MAX;
    }
}

void pid_controller_setGains(pid_controller* controller, int32_t KP, int32_t KI, int32_t KD,  uint8_t resolutionFactor) 
{
   controller->KP = KP;
   controller->KI = KI;
   controller->KD = KD;
    
   controller->resolutionFactor = resolutionFactor;
}

void computerErrors(pid_controller* ctrl) 
{
    ctrl->error = (ctrl->controlValue) - (ctrl->setpoint);
    
    //Compute difference, but not on the first time
    if(ctrl->firstTime) 
    {
        ctrl->firstTime = 0;
    }
    else
    {
        /*  Important assumption:
                PSoC compiler (Keil) makes bit shifting work like 
                multiplying / dividing by 2 for signed integers                 */
        
        //put a low pass on the difference: (3*current + 1*prev) / 4 
        ctrl->errorDifference = (3*(ctrl->error - ctrl->errorPrevious) + ctrl->errorDifference) >> 2;
        
        //Trapezoidal integration (prev + current) / 2
        //could do something fancier if rounding from division introduces too much error
        ctrl->errorSum += (ctrl->error + ctrl->errorPrevious) >> 1;
    }
    return;
}

int64_t computePidSum(pid_controller* ctrl) 
{
    return \
        (ctrl->error * ctrl->KP +
         ctrl->errorSum * ctrl->KI + 
         ctrl->errorDifference * ctrl->KD) >> ctrl->resolutionFactor;
}

void imposeControlValueMax(pid_controller* ctrl, int64_t* controller_output) 
{
    //Assumption is that outputting result of 0 will cause controlValue to tend toward 0
    if(ctrl->controlValue > ctrl->controlValueMax || ctrl->controlValue < -1*ctrl->controlValueMax)
    {
        *controller_output = 0;
    }
    else if(ctrl->controlValue > ctrl->controlValueLimitingThreshold) 
    {
        *controller_output = (*controller_output) * (ctrl->controlValueMax - ctrl->controlValue) / (ctrl->controlValueMax - ctrl->controlValueLimitingThreshold);
    }
    else if(ctrl->controlValue < -1*ctrl->controlValueLimitingThreshold)
    {
        *controller_output = (*controller_output) * (ctrl->controlValueMax + ctrl->controlValue) / (ctrl->controlValueMax - ctrl->controlValueLimitingThreshold);        
    } 
}

inline int64_t imposeMaxMagnitude(int64_t value, int64_t max) 
{
    if(value > max) value = max;
    if(value < -1*max) value = -1*max;
    return value;
}

int32_t pid_controller_compute(pid_controller* ctrl) 
{
    computerErrors(ctrl);
    
    //could put this max-out on the product of i*sum
    if(ctrl->settings & USE_ERROR_SUM_MAX)
    {
        ctrl->errorSum = imposeMaxMagnitude(ctrl->errorSum, ctrl->errorSumMax);
    }
    
    int64_t result = computePidSum(ctrl);
    
    if(ctrl->settings & USE_CONTROL_VALUE_MAX) 
    {
        imposeControlValueMax(ctrl, &result);
    }
    if(ctrl->settings & USE_OUTPUT_MAX)
    {
        result = imposeMaxMagnitude(result, ctrl->outputMax);
    }
 
    return result;
}

int32_t pid_controller_computeWithFeedForward(pid_controller* ctrl, int32_t feedForward) 
{
    computerErrors(ctrl);
    
    //could put this max-out on the product of i*sum
    if(ctrl->settings & USE_ERROR_SUM_MAX)
    {
        ctrl->errorSum = imposeMaxMagnitude(ctrl->errorSum, ctrl->errorSumMax);
    }
    
    int64_t result = computePidSum(ctrl);
    
    if(ctrl->settings & USE_CONTROL_VALUE_MAX) 
    {
        imposeControlValueMax(ctrl, &result);
    }
    
    result += feedForward;
    
    if(ctrl->settings & USE_OUTPUT_MAX)
    {
        result = imposeMaxMagnitude(result, ctrl->outputMax);
    }  
    return result;
}


