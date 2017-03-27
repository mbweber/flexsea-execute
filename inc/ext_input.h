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
	[This file] ext_input: External Input Devices/Sensors
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_EXT_IN_H
#define INC_EXT_IN_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
#include "flexsea_global_structs.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern struct enc_s encoder;	
extern uint16 ext_strain[6];

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_qei(void);
void qei_write(int32 enc);
int32 qei_read(void);
int32 refresh_enc_control(void);
int32 refresh_enc_display(void);
int16 get_analog_pos(void);

int strain_6ch_read(uint8_t internal_reg_addr, uint8_t *pData, uint16 length);
void strain_amp_6ch_test_code_blocking(void);
void strain_6ch_bytes_to_words(uint8_t *buf);
void get_6ch_strain(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define QUAD1_INIT				0	//Initial value, quadrature encoder

//Sensor commutation
#define HALL_PHYSICAL			0
#define HALL_VIRTUAL			1

//6-ch Strain Amplifier:
//(this has to match what's in amp_6_ch preipherals.h!)

#define I2C_SLAVE_ADDR_6CH		0x66	//I'm assuming this is 7bits

//EZI2C Buffer:
#define EZI2C_WBUF_SIZE			8
#define EZI2C_RBUF_SIZE			12
#define EZI2C_BUF_SIZE			(EZI2C_WBUF_SIZE + EZI2C_RBUF_SIZE)

//EZI2C Shared memory locations:
#define MEM_W_CONTROL1			0
#define MEM_W_CONTROL2			1
#define MEM_W_OFFS_CH1			2
#define MEM_W_OFFS_CH2			3
#define MEM_W_OFFS_CH3			4
#define MEM_W_OFFS_CH4			5
#define MEM_W_OFFS_CH5			6
#define MEM_W_OFFS_CH6			7
#define MEM_R_CH1_H				8
#define MEM_R_CH1_L				9
#define MEM_R_CH2_H				10
#define MEM_R_CH2_L				11
#define MEM_R_CH3_H				12
#define MEM_R_CH3_L				13
#define MEM_R_CH4_H				14
#define MEM_R_CH4_L				15
#define MEM_R_CH5_H				16
#define MEM_R_CH5_L				17
#define MEM_R_CH6_H				18
#define MEM_R_CH6_L				19

//****************************************************************************
// Structure(s)
//****************************************************************************	
	
#endif	//INC_EXT_IN_H
