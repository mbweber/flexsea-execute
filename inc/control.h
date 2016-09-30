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
	
#ifndef INC_CONTROL_H
#define INC_CONTROL_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern volatile struct ctrl_s ctrl;
extern volatile struct in_control_s in_control;
	
//****************************************************************************
// Prototype(s):
//****************************************************************************	

void control_strategy(unsigned char strat);
void init_ctrl_data_structure(void);
int32 motor_position_pid(int32 wanted_pos, int32 actual_pos);
int32 motor_position_pid_ff_1(int32 wanted_pos, int32 actual_pos, int32 ff);
int32 motor_current_pid(int32 wanted_curr, int32 measured_curr);
int32 motor_current_pid_3(int32 wanted_curr, int32 measured_curr);
extern inline int32 motor_current_pid_2(int32 wanted_curr, int32 measured_curr);
int motor_impedance_encoder(int wanted_pos, int new_enc_count);
void impedance_controller(struct as504x_s *);
void in_control_combine(void);
void in_control_get_pwm_dir(void);

void motor_cancel_damping_test_code_blocking(void);

//****************************************************************************
// Definition(s):
//****************************************************************************	

//Position controller
#define POS_PWM_LIMIT			1920				//96%
#define MAX_CUMULATIVE_ERROR	1000
#define GAIN_P					0					//Default value - will change at runtime
#define GAIN_I					0					//Idem
#define GAIN_D					0					//Idem
	
//Current controller
//#define CURRENT_ZERO			((int32)2048)		//Should be calibrated board by board
#define CURRENT_SPAN			((int32)1850)		//Variation from zero (technically, that's SPAN/2)
#define CURRENT_POS_LIMIT		CURRENT_SPAN
#define CURRENT_NEG_LIMIT		(-CURRENT_SPAN)

//Impedance controller
#define B1 						2898195LL 			// 3rd order Butterworth filter numerator constants
#define B2 						8694584LL
#define B3 						8694584LL
#define B4 						2898195LL
#define A1 						1000000000LL 		// 3rd order Butterworth filter denominator constants
#define A2 						-2374094744LL
#define A3  					1929355669LL
#define A4 						-532075368LL

//Nickname for the controller gains:
#define I_KP					g0
#define I_KI					g1
#define I_KD					g2
#define P_KP					g0
#define P_KI					g1
#define P_KD					g2
#define Z_K						g0
#define Z_B						g1
#define Z_I						g2

//Current controller tracking test:
#define PI						3.14159265
#define STEPS					50
#define STEP					(PI/STEPS)
#define DELAY					100

//****************************************************************************
// Structure(s)
//****************************************************************************	

	
#endif	//INC_MOTOR_H
	