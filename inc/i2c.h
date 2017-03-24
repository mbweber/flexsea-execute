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
	[This file] i2c: I2C functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_I2C_H
#define INC_I2C_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern uint8_t i2c_last_request;
extern volatile uint8_t i2c_0_r_buf[24];

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void i2c_0_fsm(void);
void init_i2c_0(void);
void init_i2c_1(void);
int i2c0_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *pdata, uint16 length);
uint8_t I2C_0_MasterWriteByteTimeOut(uint8_t theByte, uint32 timeout);
void assign_i2c_data(uint8_t *newdata);

//****************************************************************************
// Definition(s):
//****************************************************************************

//ISR reading of I2C0 sensors (IMU, Strain Gauge, AS5048B, etc.):
#define I2C_RQ_GYRO				1
#define I2C_RQ_ACCEL			2
#define I2C_RQ_AS5048B			3
#define I2C_RQ_EXT_STRAIN		4

//****************************************************************************
// Structure(s)
//****************************************************************************


#endif	//INC_I2C_H
