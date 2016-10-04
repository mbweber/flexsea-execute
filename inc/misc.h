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
	[This file] misc: when it doesn't belong in any another file, it ends up 
	here...
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/
	
#ifndef INC_MISC_H
#define INC_MISC_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************
	
extern volatile uint8 t1_100us_flag;
extern volatile uint8 t1_time_share, t1_new_value;
	
extern uint8 adc_sar1_flag;	
extern volatile uint8 data_ready_485;
extern volatile uint8 data_ready_usb;
extern volatile uint8 adc_delsig_flag;

extern uint16 last_as5047_word;	    
extern int32 angle_read_counter; 
extern int32 last_angle_read_gap;
    
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void test_code_blocking(void);	
void test_code_non_blocking(void);
uint8 timebase_1s(void);
uint8 timebase_100ms(void);

void timing_test_blocking(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define SDELAY	5
	
#endif	//INC_MISC_H
	