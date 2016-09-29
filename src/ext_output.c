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
	[Contributors]
*****************************************************************************
	[This file] ext_output: External Output Devices/Actuators
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "ext_output.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8 clutch_pwm = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************

//Configuration for the clutch
void init_pwro(void)
{
	//PWM2: Clutch / Power Output
	PWM_4_Start();
	PWM_4_WriteCompare(0);	//Start at 0%
}

//PWM, power output
void pwro_output(uint8 value)
{
	clutch_pwm = value;
	PWM_4_WriteCompare(clutch_pwm);
}

//Returns the PWM value of the power output
uint8 read_pwro(void)
{
	return clutch_pwm;
}

void test_pwro_output_blocking(void)
{
	uint8 pwm = 0;
	
	while(1)
	{
		pwm++;
		pwro_output(pwm);	
		CyDelay(1);
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************


//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

