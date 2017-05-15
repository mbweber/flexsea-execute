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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] JF Duval, Jake Mooney
*****************************************************************************
	[This file] sensor_commut: Angle Sensor Motor Commutation
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_SENSOR_COMMUT_H
#define INC_SENSOR_COMMUT_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern int findingpoles;

extern uint8_t measure_motor_resistance;

extern int16 phaseAcoms[2048]; 
extern int16 phaseBcoms[2048]; 
extern int16 phaseCcoms[2048]; 

extern int32_t PWM_A_Value;
extern int32_t PWM_B_Value;
extern int32_t PWM_C_Value;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void find_poles(void);
void load_eeprom_to_angles(void);
void fill_comm_tables(int32,int16_t *);
void sensor_sin_commut(int16, int32);

void test_sinusoidal_blocking(void);
int get_sin_profile(double, double);

void calc_motor_L(void);

//****************************************************************************
// Definition(s):
#define NUMPOLES        126
#define SHIFT_G         0
#define PWM_MAX         990
#define PWM_DEAD        20 //dead time caused by the PWM module + extra needed to stop wishing sound
#define PWM_AMP         495//(2000-PWM_DEAD)/4
#define MAX_ENC         16383
//#define PWM_DEAD2		41 //dead time caused by the opening and closing of the FETS
//****************************************************************************

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_SENSOR_COMMUT_H
