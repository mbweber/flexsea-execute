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
	[This file] local_comm: Communication functions, board specific
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-03-06 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_LOCAL_COMM_H
#define INC_LOCAL_COMM_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>

//***************************************************************************
// Shared variable(s)
//****************************************************************************



//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void initLocalComm(void);
void parseMasterCommands(uint8_t *new_cmd);
void sendMasterDelayedResponse(void);

//****************************************************************************
// Definition(s):
//****************************************************************************


//****************************************************************************
// Structure(s)
//****************************************************************************


#endif	//INC_LOCAL_COMM_H
