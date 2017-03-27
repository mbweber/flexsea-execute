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
	[Contributors] Luke Mooney
*****************************************************************************
	[This file] motor: motor control functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_MOTOR_H
#define INC_MOTOR_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

void init_motor(void);	
void motor_open_speed_1(int32 pwm_duty);
void motor_open_speed_2(int16 pwm_duty, int sign);

void motor_fixed_pwm_test_code_blocking(int spd);
void motor_fixed_pwm_test_code_non_blocking(int spd);

void motor_stepper_test_blocking_1(int spd);
void motor_stepper_test_init(int spd);
void motor_stepper_test_runtime(int div);

void initDmaPwmCompare(void);
void setDmaPwmCompare(uint16_t a, uint16_t b, uint16_t c);

//****************************************************************************
// Definition(s):
//****************************************************************************

//PWM limits
#define MAX_PWM						1000
#define MIN_PWM						-MAX_PWM
#define P1_DEADTIME					30					//Make sure that it matches the hardware setting!
#define PWM1DC(x)					MAX(x, (P1_DEADTIME+2))
#define PWM2DC(x)					MAX(((x - P1_DEADTIME)>>1), 10)

//Common definitions, DMA_Px:
#define DMA_PX_BYTES_PER_BURST 		2
#define DMA_PX_REQUEST_PER_BURST 	1
#define DMA_PX_SRC_BASE 			(CYDEV_SRAM_BASE)
#define DMA_PX_DST_BASE 			(CYDEV_PERIPH_BASE)
	
//****************************************************************************
// Structure(s)
//****************************************************************************	

#endif	//INC_MOTOR_H
