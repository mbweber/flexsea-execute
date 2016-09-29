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
	[This file] demo: Demonstration Test Code
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_DEMO_H
#define INC_DEMO_H

//****************************************************************************
// Include(s)
//****************************************************************************		
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

	
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************	

//****************************************************************************
// Definition(s):
//****************************************************************************	

//CSEA Knee will go up and down. Trajectory calculated by Trapez, feeding a position PI.
#define CSEA_UP 		72000
#define CSEA_DOWN 		11500
#define MARGIN			375
#define SPD_UP			7000000
#define ACC_UP			4000000
#define SPD_DOWN		7000000
#define ACC_DOWN		4500000
#define TICK_TO_ANGLE	672
#define FF_GAIN			10
	
//****************************************************************************
// Structure(s)
//****************************************************************************	

	
#endif	//INC_DEMO_H
	