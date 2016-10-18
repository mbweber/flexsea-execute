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

extern uint8 measure_motor_resistance;

extern uint16 phaseAcoms[2048]; 
extern uint16 phaseBcoms[2048]; 
extern uint16 phaseCcoms[2048]; 
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void motor_spi_block_commutation(int angle);
void motor_spi_block_commutation_triangletest();
void motor_spi_findpoles();
void find_poles(void);
void find_poles_blocking(void);
void find_poles2(void);
void non_blocking_step_test(void);
void non_blocking_sin_commut(int16 pwm);
void load_eeprom_to_angles(void);
void fill_comm_tables(int32);
void sensor_sin_commut(int16, int32);

void test_sinusoidal_blocking(void);
int get_sin_profile(int32, int32);

//****************************************************************************
// Definition(s):
#define NUMPOLES        126
#define SHIFT_G         0
#define PWM_DEAD        90 //dead time causes by the PWM module
#define MAX_ENC         16383
//#define PWM_DEAD2       41 //dead time caused by the opening and closing of the FETS
//****************************************************************************

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_SENSOR_COMMUT_H
