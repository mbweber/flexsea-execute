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
	[This file] main: FlexSEA-Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

//****************************************************************************
// Include(s)
//****************************************************************************

#include <project.h>
#include <DieTemp_1.h>
#include <math.h>
#include "stdio.h"
#include "main_fsm.h"
#include "serial.h"
#include "i2c.h"
#include "motor.h"
#include "misc.h"
#include "ui.h"
#include "rgb_led.h"
#include "trapez.h"
#include "imu.h"
#include "analog.h"
#include "usb.h"
#include "strain.h"
#include "safety.h"
#include "peripherals.h"
#include "control.h"
#include "sensor_commut.h"
#include "ext_input.h"
#include "ext_output.h"
#include "gen_waveform.h"
#include "mem_angle.h"	
#include "i2t-current-limit.h"
#include "demo.h"
#include "flexsea_board.h"
#include "../flexsea-user/inc/user-ex.h"
#include "../flexsea-system/inc/flexsea_system.h"
#include "../flexsea-comm/inc/flexsea.h"
#include "../flexsea-system/test/flexsea-system_test-all.h"	

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern int global_variable_1;
extern int global_variable_2;
extern int global_variable_3;
extern int global_variable_4;
extern int global_variable_5;
extern int global_variable_6;
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

int main(void);

#if defined (__GNUC__)
    asm (".global _printf_float");
#endif

//****************************************************************************
// Definition(s):
//****************************************************************************

#endif // MAIN_H_
