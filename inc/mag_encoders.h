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

void init_as5047(void);
uint16 as5047_read_single_isr(uint16 reg);
void init_angle_timer(void);

int as5048b_read(uint8_t internal_reg_addr, uint8_t *pData, uint16 length);
void as5048b_test_code_blocking(void);
void get_as5048b_position(void);

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

extern struct as504x_s as5047, as5048b;
extern volatile uint8_t spi_isr_state;
extern uint16 spidata_miso[];
extern uint16 spidata_mosi2[];
extern uint16 as5047_angle;
extern int32_t spi_read_flag;
extern volatile uint16 as5047_empty_read;

//****************************************************************************
// Definition(s):
//****************************************************************************

//AS5047 Magnetic encoder (SPI):

	//Registers:
#define AS5047_REG_NOP			0
#define AS5047_REG_ERRFL		0x0001
#define AS5047_REG_PROG			0x0003
#define AS5047_REG_DIAAGC		0x3FFC
#define AS5047_REG_MAG			0x3FFD
#define AS5047_REG_ANGLEUNC		0x3FFE
#define AS5047_REG_ANGLECOM		0x3FFF

	//Commands & parity:	
#define AS5047_READ				0x4000     	//OR it with your word
#define AS5047_WRITE			0xBFFF     	//AND it with your word
#define PARITY_0				0x7FFF     	//AND it with your word
#define PARITY_1				0x8000     	//OR it with your word
#define WORDS_IN_FRAME			7
#define AS5047_ERR_FRAME		0x4000		//Error frame indicator

#define SPI_TX_MAX_INDEX		1

//AS5048B Magnetic Encoder (I2C):

#define I2C_ADDR_AS5048B		0b1000000	//A1 & A2 forced low, default address
#define AD5048B_REG_AGC			250
#define AD5048B_REG_DIAG		251
#define AD5048B_REG_MAG_H		252
#define AD5048B_REG_MAG_L		253
#define AD5048B_REG_ANGLE_H		254
#define AD5048B_REG_ANGLE_L		255

#endif	//INC_MAG_ENCODERS_H
