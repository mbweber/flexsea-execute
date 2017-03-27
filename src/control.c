/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-execute' Advanced Motion Controller
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
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Elliott Rouse, Luke Mooney
*****************************************************************************
	[This file] control: Control Loops
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "control.h"
#include "motor.h"
#include "ext_input.h"
#include "user-ex.h"
#include "calibration_tools.h"
#include "sensor_commut.h"
#include "trapez.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Main data structure for all the controllers:
volatile struct ctrl_s ctrl;

//In Control tool:
volatile struct in_control_s in_control;

//Impedance loop
int debug_var = 0;

//****************************************************************************
// Function(s)
//****************************************************************************

//Use this function to change the control strategy
void control_strategy(uint8_t strat)
{
	//Are we already using this controller?
	//Are we currently running a calibration routine?
	if(calibrationFlags || ctrl.active_ctrl == strat)
	{
		//Yes. Nothing to do, exit.
		return;
	}
	else
	{
		//Different controller
	
		//By default we place the gains to 0 before we change the strategy:
						
		//Position controller
		ctrl.position.gain.P_KP = 0;
		ctrl.position.gain.P_KI = 0;
		ctrl.position.gain.P_KD = 0;
		
		//Impedance controller
		ctrl.impedance.gain.Z_K = 0;
		ctrl.impedance.gain.Z_B = 0;
		ctrl.impedance.gain.Z_I = 0;
		
		//Current controller
		ctrl.current.gain.I_KP = 0;
		ctrl.current.gain.I_KI = 0;
		ctrl.current.gain.I_KD = 0;
		ctrl.current.setpoint_val = 0;
        ctrl.current.error_sum = 0;
		
		//To avoid a huge startup error on the Position-based controllers:
		if(strat == CTRL_POSITION)
		{
			ctrl.position.setp = refresh_enc_control();
			steps = trapez_gen_motion_1(ctrl.position.setp, ctrl.position.setp, 1, 1);
		}
		else if(strat == CTRL_IMPEDANCE)
		{
			ctrl.impedance.setpoint_val = refresh_enc_control();
			steps = trapez_gen_motion_1(ctrl.impedance.setpoint_val, ctrl.impedance.setpoint_val, 1, 1);
		}
        
        if (strat != CTRL_MEASRES)
        {
            measure_motor_resistance = 0;
        }
        else
        {
            measure_motor_resistance = 1;
        }
		
		ctrl.active_ctrl = strat;
		in_control.controller = ctrl.active_ctrl;
		
		//The user should call a set gain function at this point.
	}
}

//Starts all the parameters at 0 or Default values
void init_ctrl_data_structure(void)
{
	//No active controller:
	ctrl.active_ctrl = CTRL_NONE;
	
	//Generic controller
	ctrl.generic.gain.g0 = 0;
	ctrl.generic.gain.g1 = 0;
	ctrl.generic.gain.g2 = 0;
	ctrl.generic.gain.g3 = 0;
	ctrl.generic.gain.g4 = 0;
	ctrl.generic.gain.g5 = 0;
	ctrl.generic.actual_val = 0;
	ctrl.generic.setpoint_val = 0;
	ctrl.generic.error = 0;
	ctrl.generic.error_sum = 0;
	ctrl.generic.error_dif = 0;
	
	//Position controller
	ctrl.position.gain.g0 = 0;
	ctrl.position.gain.g1 = 0;
	ctrl.position.gain.g2 = 0;
	ctrl.position.gain.g3 = 0;
	ctrl.position.gain.g4 = 0;
	ctrl.position.gain.g5 = 0;
	ctrl.position.pos = 0;
	ctrl.position.setp = 0;
	ctrl.position.error = 0;
	ctrl.position.error_sum = 0;
	ctrl.position.error_dif = 0;
	
	//Impedance controller
	ctrl.impedance.gain.g0 = 0;
	ctrl.impedance.gain.g1 = 0;
	ctrl.impedance.gain.g2 = 0;
	ctrl.impedance.gain.g3 = 0;
	ctrl.impedance.gain.g4 = 0;
	ctrl.impedance.gain.g5 = 0;
	ctrl.impedance.actual_val = 0;
    ctrl.impedance.actual_vel = 0;
	ctrl.impedance.setpoint_val = 0;
	ctrl.impedance.error = 0;
	ctrl.impedance.error_sum = 0;
	ctrl.impedance.error_dif = 0;
	
	//Current controller
	ctrl.current.gain.g0 = 0;
	ctrl.current.gain.g1 = 0;
	ctrl.current.gain.g2 = 0;
	ctrl.current.gain.g3 = 0;
	ctrl.current.gain.g4 = 0;
	ctrl.current.gain.g5 = 0;
	ctrl.current.actual_val = 0;
	ctrl.current.setpoint_val = 0;
	ctrl.current.error = 0;
	ctrl.current.error_sum = 0;
	ctrl.current.error_dif = 0;
}

//ToDo move:
#define MAX_ERR_SUM	400000
#define PWM_SAT		30000	

//Motor position controller - non blocking
int32 motor_position_pid(int32 wanted_pos, int32 actual_pos)
{
	int32 p = 0, i = 0, d = 0;
	int32 pwm = 0;

	//Position values:
	ctrl.position.pos = actual_pos;
	ctrl.position.setp = wanted_pos;
	in_control.actual_val = ctrl.position.pos;
	in_control.setp = ctrl.position.setp;
	
	//Errors:
	ctrl.position.error_prev = ctrl.position.error;
	ctrl.position.error = ctrl.position.setp - ctrl.position.pos;
	in_control.error = ctrl.position.error;
	ctrl.position.error_sum = ctrl.position.error_sum + ctrl.position.error;
	ctrl.position.error_dif = (ctrl.position.error_dif << 1) + \
							6*(ctrl.position.error - ctrl.position.error_prev);
	ctrl.position.error_dif = ctrl.position.error_dif >> 3;	
	
	//Saturate cumulative error
	if(ctrl.position.error_sum >= MAX_ERR_SUM)
		ctrl.position.error_sum = MAX_ERR_SUM;
	if(ctrl.position.error_sum <= -MAX_ERR_SUM)
		ctrl.position.error_sum = -MAX_ERR_SUM;
	
	//Proportional term
	p = (ctrl.position.gain.P_KP * ctrl.position.error) / 100;
	in_control.r[0] = p;
	//Integral term
	i = (ctrl.position.gain.P_KI * ctrl.position.error_sum) / 100;
	in_control.r[1] = i;
	//Differential term:
	d = (ctrl.position.gain.P_KD * ctrl.position.error_dif) / 100;
	in_control.r[2] = d;
	
	//Output
	pwm = (p + i + d);		//
	
	//Saturates PWM to low values
	if(pwm >= PWM_SAT)
		pwm = PWM_SAT;
	if(pwm <= -PWM_SAT)
		pwm = -PWM_SAT;
	
	motor_open_speed_1(pwm);
	in_control.output = pwm;
	
	return ctrl.position.error;
}

//Motor position controller - non blocking. PID + Feed Forward (FF)
//The FF term comes from the calling function, it's added to the output.
int32 motor_position_pid_ff_1(int32 wanted_pos, int32 actual_pos, int32 ff)
{
	int32 p = 0, i = 0, d = 0;
	int32 pwm = 0;

	//Position values:
	ctrl.position.pos = actual_pos;
	ctrl.position.setp = wanted_pos;
	
	//Errors:
	ctrl.position.error = ctrl.position.pos - ctrl.position.setp;
	ctrl.position.error_sum = ctrl.position.error_sum + ctrl.position.error;
	//ctrl.position.error_dif ToDo
	
	//Saturate cumulative error
	if(ctrl.position.error_sum >= MAX_CUMULATIVE_ERROR)
		ctrl.position.error_sum = MAX_CUMULATIVE_ERROR;
	if(ctrl.position.error_sum <= -MAX_CUMULATIVE_ERROR)
		ctrl.position.error_sum = -MAX_CUMULATIVE_ERROR;
	
	//Proportional term
	p = (ctrl.position.gain.P_KP * ctrl.position.error) / 100;
	//Integral term
	i = (ctrl.position.gain.P_KI * ctrl.position.error_sum) / 100;
	//Derivative term
	d = 0;	//ToDo
	
	//Output
	pwm = (p + i + d) + ff;		//
	
	//Saturates PWM to low values
	if(pwm >= POS_PWM_LIMIT)
		pwm = POS_PWM_LIMIT;
	if(pwm <= -POS_PWM_LIMIT)
		pwm = -POS_PWM_LIMIT;
	
	motor_open_speed_1(pwm);
	
	return ctrl.position.error;
}

//PI Current controller
//'wanted_curr' is centered at zero and is in the ±CURRENT_SPAN range
//'measured_curr' is also centered at 0
//The sign of 'wanted_curr' will change the rotation direction, not the polarity of the current (I have no control on this)
int32 motor_current_pid(int32 wanted_curr, int32 measured_curr)
{
	int curr_p = 0, curr_i = 0;
	int curr_pwm = 0;
	int sign = 0;
	unsigned int uint_wanted_curr = 0;
	int motor_current = 0;
	int32 shifted_measured_curr = 0;
	
	//Clip out of range values
	if(wanted_curr >= CURRENT_POS_LIMIT)
		wanted_curr = CURRENT_POS_LIMIT;
	if(wanted_curr <= CURRENT_NEG_LIMIT)
		wanted_curr = CURRENT_NEG_LIMIT;
		
	ctrl.current.setpoint_val = wanted_curr;
	
	//Sign extracted from wanted_curr:
	if(wanted_curr < 0)
	{
		sign = -1;
		uint_wanted_curr = -wanted_curr;
	}
	else
	{
		sign = 1;
		uint_wanted_curr = wanted_curr;
	}
	
	//At this point 'uint_wanted_curr' is always a positive value.
	//This is our setpoint.
	
	//From ADC value to motor current:
	shifted_measured_curr = measured_curr + CURRENT_ZERO;
	if(shifted_measured_curr <= CURRENT_ZERO)
	{
		//We are driving the motor (Q1 or Q3)
		motor_current = CURRENT_ZERO - shifted_measured_curr;
	}
	else
	{
		motor_current =  shifted_measured_curr - CURRENT_ZERO;
	}
	//ToDo above code seems complex for no valid reason
	
	//At this point 'motor_current' is always a positive value.
	//This is our measured value.
	
	//Error and integral of errors:
	ctrl.current.error = uint_wanted_curr - motor_current;					//Actual error
	ctrl.current.error_sum = ctrl.current.error_sum + ctrl.current.error;	//Cumulative error
	
	//Saturate cumulative error
	if(ctrl.current.error_sum >= MAX_CUMULATIVE_ERROR)
		ctrl.current.error_sum = MAX_CUMULATIVE_ERROR;
	if(ctrl.current.error_sum <= -MAX_CUMULATIVE_ERROR)
		ctrl.current.error_sum = -MAX_CUMULATIVE_ERROR;
	
	//Proportional term
	curr_p = (int) ctrl.current.gain.I_KP * ctrl.current.error;
	//Integral term
	curr_i = (int)(ctrl.current.gain.I_KI * ctrl.current.error_sum)/100;
	//Add differential term here if needed
	
	//Output
	curr_pwm = curr_p + curr_i;
	
	//Saturates PWM
	if(curr_pwm >= POS_PWM_LIMIT)
		curr_pwm = POS_PWM_LIMIT;
	if(curr_pwm <= 0)	//Should not happen
		curr_pwm = 0;
	
	//Apply PWM
	motor_open_speed_2(curr_pwm, sign);
	
	return ctrl.current.error;
}

//PI Current controller #2: speed optimized
//'wanted_curr' & 'measured_curr' are centered at zero and are in the ±CURRENT_SPAN range
//The sign of 'wanted_curr' will change the rotation direction, not the polarity of the current (I have no control on this)
inline int32 motor_current_pid_2(int32 wanted_curr, int32 measured_curr)
{
	volatile int32 curr_p = 0, curr_i = 0;
	volatile int32 curr_pwm = 0;
	int32 sign = 0;
	int32 uint_wanted_curr = 0;
	int32 motor_current = 0;
	
	//Clip out of range values
	if(wanted_curr >= CURRENT_POS_LIMIT)
		wanted_curr = CURRENT_POS_LIMIT;
	if(wanted_curr <= CURRENT_NEG_LIMIT)
		wanted_curr = CURRENT_NEG_LIMIT;		
	ctrl.current.setpoint_val = wanted_curr;
	
	//Sign extracted from wanted_curr:
	if(wanted_curr < 0)
	{
		sign = (-1)*MOTOR_ORIENTATION;
		#if(MOTOR_COMMUT == COMMUT_BLOCK)
		MotorDirection_Control = 0;		//MotorDirection_Write(0);
		#else
			//ToDo Sine
		#endif	//(MOTOR_COMMUT == COMMUT_BLOCK)
		uint_wanted_curr = -wanted_curr;
	}
	else
	{
		sign = (1)*MOTOR_ORIENTATION;
		#if(MOTOR_COMMUT == COMMUT_BLOCK)
		MotorDirection_Control = 1;		//MotorDirection_Write(1);
		#else
			//ToDo Sine
		#endif	//(MOTOR_COMMUT == COMMUT_BLOCK)
		uint_wanted_curr = wanted_curr;
	}
	
	//At this point 'uint_wanted_curr' is always a positive value.
	//This is our setpoint.
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	
	int32 shifted_measured_curr = 0;
		
	//From ADC value to motor current:
	shifted_measured_curr = measured_curr + CURRENT_ZERO;
	if(shifted_measured_curr <= CURRENT_ZERO)
	{
		//We are driving the motor (Q1 or Q3)
		motor_current = CURRENT_ZERO - shifted_measured_curr;
	}
	else
	{
		motor_current =  shifted_measured_curr - CURRENT_ZERO;
	}
	//ToDo above code seems complex for no valid reason
	
	#else
		
	motor_current = measured_curr;
		
	#endif
	
	//At this point 'motor_current' is always a positive value.
	//This is our measured value.
	
	//Error and integral of errors:
	ctrl.current.error = uint_wanted_curr - motor_current;					//Actual error
	ctrl.current.error_sum = ctrl.current.error_sum + ctrl.current.error;	//Cumulative error
	
	//Saturate cumulative error
	if(ctrl.current.error_sum >= MAX_CUMULATIVE_ERROR)
		ctrl.current.error_sum = MAX_CUMULATIVE_ERROR;
	if(ctrl.current.error_sum <= -MAX_CUMULATIVE_ERROR)
		ctrl.current.error_sum = -MAX_CUMULATIVE_ERROR;
	
	//Proportional term
	curr_p = (int) (ctrl.current.gain.I_KP * ctrl.current.error) / 100;
	//Integral term
	curr_i = (int)(ctrl.current.gain.I_KI * ctrl.current.error_sum) / 100;
	//Add differential term here if needed
	//In both cases we divide by 100 to get a finer gain adjustement w/ integer values.
	
	//Output
	curr_pwm = curr_p + curr_i;
	
	//Saturates PWM
	if(curr_pwm >= POS_PWM_LIMIT)
		curr_pwm = POS_PWM_LIMIT;
	if(curr_pwm <= 0)	//Should not happen
		curr_pwm = 0;
	
	//Apply PWM
	//motor_open_speed_2(curr_pwm, sign);
	//Integrated to avoid a function call and a double saturation:
	
	//Write duty cycle to PWM module (avoiding double function calls)
	curr_pwm = PWM1DC(curr_pwm);
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	CY_SET_REG16(PWM_1_COMPARE1_LSB_PTR, (uint16)curr_pwm);				//PWM_1_WriteCompare1((uint16)curr_pwm);
	CY_SET_REG16(PWM_1_COMPARE2_LSB_PTR, (uint16)(PWM2DC(curr_pwm)));	//PWM_1_WriteCompare2((uint16)((curr_pwm >> 1) + 1));	
	//Compare 2 can't be 0 or the ADC won't trigger => that's why I'm adding 1
	#else
	exec1.sine_commut_pwm = MOTOR_ORIENTATION * curr_pwm;
	#endif	//(MOTOR_COMMUT == COMMUT_BLOCK)
	
	//ToDo: where's the sign applied???
	
	return ctrl.current.error;
}

//PI Current controller #3: for sinusoidal commutation
//'wanted_curr' & 'measured_curr' are centered at zero and are in the ±CURRENT_SPAN range
//The sign of 'wanted_curr' will change the rotation direction, not the polarity of the current (I have no control on this)
inline int32 motor_current_pid_3(int32 wanted_curr, int32 measured_curr)
{
	int32_t sign = 0;
    //Clip out of range values
	//if(wanted_curr >= CURRENT_POS_LIMIT)
	//	wanted_curr = CURRENT_POS_LIMIT;	
	
	//Error and integral of errors:
	ctrl.current.error = (wanted_curr - measured_curr);					//Actual error
	ctrl.current.error_sum = ctrl.current.error_sum + ctrl.current.gain.I_KI*ctrl.current.error;	//Cumulative error
	
	//Saturate cumulative error
	if(ctrl.current.error_sum >= MAX_CUM_CURRENT_ERROR)
		ctrl.current.error_sum = MAX_CUM_CURRENT_ERROR;
	if(ctrl.current.error_sum <= -MAX_CUM_CURRENT_ERROR)
		ctrl.current.error_sum = -MAX_CUM_CURRENT_ERROR;	

	//Proportional term
	volatile int32 curr_p = (int) ((ctrl.current.gain.I_KP * ctrl.current.error)>>8);
	//Integral term
	volatile int32 curr_i = (int)((ctrl.current.error_sum)>>8);
	//Add differential term here if needed
	//In both cases we divide by 100 to get a finer gain adjustement w/ integer values.

	//Output
	volatile int32 curr_pwm = curr_p + curr_i+(*exec1.enc_ang_vel)*366/10*0+((wanted_curr*8)/43);
	
	#if(MOTOR_COMMUT == COMMUT_SINE) 

        /*
	    //sine_commut_pwm should be from 1024 to -1024
	    if (curr_pwm > 1023)
	    curr_pwm = 1023;
	    if (curr_pwm < -1023)
	    curr_pwm = -1023;
	    exec1.sine_commut_pwm = MOTOR_ORIENTATION*curr_pwm;
        */
        
        motor_open_speed_1(curr_pwm);
	
	#endif
		
	#if(MOTOR_COMMUT == COMMUT_BLOCK)			
		//Sign extracted from wanted_curr:
		if(wanted_curr < 0)
		{
			curr_pwm = -curr_pwm;
			MotorDirection_Control = 0;		//MotorDirection_Write(0);
		}
		else
		{
			MotorDirection_Control = 1;		//MotorDirection_Write(1);
		}

		//Saturates PWM
		if(curr_pwm >= POS_PWM_LIMIT)
			curr_pwm = POS_PWM_LIMIT;
		if(curr_pwm <= -POS_PWM_LIMIT)
			curr_pwm = POS_PWM_LIMIT;

		//Write duty cycle to PWM module (avoiding double function calls)
		curr_pwm = PWM1DC(curr_pwm);

		CY_SET_REG16(PWM_1_COMPARE1_LSB_PTR, (uint16)curr_pwm);	
		CY_SET_REG16(PWM_1_COMPARE2_LSB_PTR, (uint16)(PWM2DC(curr_pwm)));
		//Compare 2 can't be 0 or the ADC won't trigger => that's why I'm adding 1
	#endif
        
	return ctrl.current.error;    
}

// Impedance controller -- EJ Rouse, 8/11/14
// There will be filter transients for first few iterations -- maybe turn loops off for 100 ms?
// Variables created: stiffness, damping, prev_enc_count
int motor_impedance_encoder(int wanted_pos, int new_enc_count)
{
	// Initialize vars
	int i_k = 0, i_b = 0;	
	static long long prev_enc_count = 0, prev_vel1 = 0, prev_vel2 = 0, prev_vel3 = 0;
	static long long prev_filt1 = 0, prev_filt2 = 0, prev_filt3 = 0; 
	long long current_vel = 0, filt_vel = 0, current_val = 0;
	static int enc_t0 = 0, enc_tm1 = 0, enc_tm2 = 0, enc_tm3 = 0, enc_tm4 = 0, enc_tm5 = 0, enc_tm6 = 0, enc_tm7 = 0, enc_tm8 = 0, enc_tm9 = 0;
	long long motor_direction = new_enc_count - prev_enc_count;
	int modifier = 0;
	
	//Test code:
	if(motor_direction <= 0)
	{
		//MotorDirection_Write(0);
		//modifier = 50;
	}
	else
	{
		//MotorDirection_Write(1);
		modifier = 0;
	}
	//End of test code

	ctrl.impedance.error = new_enc_count - wanted_pos;		//Actual error
	in_control.setp = wanted_pos;
	in_control.actual_val = new_enc_count;
	in_control.error = ctrl.impedance.error;
	
	//Current for stiffness term. Gain factor depends on the encoder count.
	#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
		i_k = ctrl.impedance.gain.Z_K * (ctrl.impedance.error >> 4);
	#else
		i_k = ctrl.impedance.gain.Z_K * (ctrl.impedance.error >> 8);
	#endif
	
	
	//Velocity measured on n cycles:
	enc_tm9 = enc_tm8; enc_tm8 = enc_tm7; enc_tm7 = enc_tm6; enc_tm6 = enc_tm5; enc_tm5 = enc_tm4; enc_tm4 = enc_tm3; enc_tm3 = enc_tm2; enc_tm2 = enc_tm1; enc_tm1 = enc_t0;
	enc_t0 = new_enc_count;
	
	//Damping term
	//Difference velocity
	//current_vel = (enc_t0 - enc_tm4);
	//debug_var = current_vel;

 	filt_vel = (57*enc_t0)/220 + (73*enc_tm1)/660 - enc_tm2/264 - (37*enc_tm3)/440 - (43*enc_tm4)/330 - (47*enc_tm5)/330 - (53*enc_tm6)/440 - (17*enc_tm7)/264 + (17*enc_tm8)/660 + (3*enc_tm9)/20; // Derivative method that estimates best fit second order polynomial and calcualtes the derivative numerically for t = 0
 	debug_var = filt_vel;
	
 	i_b = ctrl.impedance.gain.Z_B * (filt_vel >> 5);
	i_b += modifier;

	//Output
	current_val = (i_k + i_b);		// Impedance command, motor current in terms of effort (A)
	
	// Changes variables for derivative filtering
	prev_enc_count = new_enc_count;
	prev_vel3 = prev_vel2;
	prev_vel2 = prev_vel1;
	prev_vel1 = current_vel;
	prev_filt3 = prev_filt2;
	prev_filt2 = prev_filt1;
	prev_filt1 = filt_vel;

	ctrl.current.setpoint_val = (int32)current_val;
	in_control.output = ctrl.current.setpoint_val;

	return ctrl.impedance.error;
}

void impedance_controller(void)
{
    int32 spring_torq; 
    int32 damping_torq;

    spring_torq = ((ctrl.impedance.setpoint_val-ctrl.impedance.actual_val)*ctrl.impedance.gain.g0)>>4;
    damping_torq = (-1*(ctrl.impedance.actual_vel)*ctrl.impedance.gain.g1);
    
    ctrl.current.setpoint_val = (spring_torq+damping_torq);
}

//in_control.combined = [CTRL2:0][MOT_DIR][PWM]
void in_control_combine(void)
{
	uint16_t tmp = 0;
	
	tmp = ((in_control.controller & 0x03) << 13) | ((in_control.mot_dir & 0x01) << 12) | (in_control.pwm & 0xFFF);
	in_control.combined = tmp;
}

//Reads the PWM and MOTOR_DIR values from hardware:
void in_control_get_pwm_dir(void)
{
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	in_control.pwm = PWM_1_ReadCompare1();
	in_control.mot_dir = MotorDirection_Read();
	#else
	//ToDo Sine
	in_control.pwm = 0;
	in_control.mot_dir = 0;
	#endif	//#if(MOTOR_COMMUT == COMMUT_BLOCK)
}

//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************
