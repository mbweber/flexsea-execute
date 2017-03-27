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
	[This file] mag_encoders: AS504x Magnetic Encoders
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_MAG_ENCODERS_H
#define INC_MAG_ENCODERS_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
//#include "../../flexsea-system/inc/flexsea_system.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_angle_timer(void);

void update_counts_since_last_ang_read(struct as504x_s *as504x);
void reset_ang_counter(struct as504x_s *);
void init_angsense(struct angsense_s *as);
void init_as504x(struct as504x_s *as504x);

void update_as504x(int32_t ang, struct as504x_s *as504x);
void update_as504x_ang(int32_t ang, struct as504x_s *as504x);
void update_as504x_vel(struct as504x_s *as504x);


//****************************************************************************
// Shared Variable(s):
//****************************************************************************


//****************************************************************************
// Definition(s):
//****************************************************************************

#endif	//INC_MAG_ENCODERS_H
