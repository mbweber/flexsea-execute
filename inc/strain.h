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
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] strain: strain gauge amplifier
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_STRAINGAUGE_H
#define INC_STRAINGAUGE_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//Onboard:
extern struct strain_s strain1;
extern uint16_t adc_strain_filtered;	
extern volatile uint16_t adc_strain;
extern volatile uint16 adc_delsig_dma_array[8];

//External:
extern uint16 ext_strain[6];
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

void init_strain(void);
void strain_config(uint8_t offs, uint8_t gain);
uint16 strain_read(void);
uint16 strain_filter(void);
void strain_test_blocking(void);
void dma_2_config(void);
uint16 strain_filter_dma(void);

int strain_6ch_read(uint8_t internal_reg_addr, uint8_t *pData, uint16 length);
void strain_amp_6ch_test_code_blocking(void);
void strain_6ch_bytes_to_words(uint8_t *buf);
void get_6ch_strain(void);
uint8_t compressAndSplit6ch(uint8_t *buf, uint16 ch0, uint16 ch1, uint16 ch2, \
							uint16 ch3, uint16 ch4, uint16 ch5);
void unpackCompressed6ch(uint8_t *buf, uint16 *v0, uint16 *v1, uint16 *v2, \
							uint16 *v3, uint16 *v4, uint16 *v5);
void compress6chTestCodeBlocking(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//I2C Addresses - 7bits convention
#define I2C_POT_ADDR			0b0101000

//MCP4661
#define MCP4661_REG_RAM_W0		0x00
#define MCP4661_REG_RAM_W1		0x10
#define MCP4661_REG_EEP_W0		0x20
#define MCP4661_REG_EEP_W1		0x30
#define MCP4661_REG_TCON		0x40
#define MCP4661_REG_STATUS		0x50
#define MCP4661_WRITE_CMD		0x00
#define MCP4661_READ_CMD		0x0A
#define MCP4661_CONFIG			0xFF 	//(POR default)
//W0 is gain, W1 is offset

#define STRAIN_OFFSET			MCP4661_REG_RAM_W1
#define STRAIN_GAIN				MCP4661_REG_RAM_W0

//Default values (experimental, expected to change)
#define STRAIN_DEFAULT_OFFSET	125 //0-255
#define STRAIN_DEFAULT_GAIN		50 //0-255 //0 = gain of 5, 255 = gain of 1000, increases linearly
                                   //50 = 200
                                   //strain output is 76.294 uV per click

//Strain filtering:
#define STRAIN_BUF_LEN			6
#define STRAIN_SHIFT			2	//Needs to match STRAIN_BUF_LEN

//DMA Bytes per transfer (16bits values = 2 bytes/word)
#define DMA2_BYTES_PER_XFER		(2*STRAIN_BUF_LEN)

//6-ch Strain Amplifier:
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
// Structure(s):
//****************************************************************************

#endif	//INC_STRAINGAUGE_H
