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

extern struct strain_s strain1;
extern uint16_t adc_strain_filtered;	
extern volatile uint16_t adc_strain;
extern volatile uint16 adc_delsig_dma_array[8];
	
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

uint8_t compressAndSplit6ch(uint8_t *buf, uint16 ch0, uint16 ch1, uint16 ch2, \
							uint16 ch3, uint16 ch4, uint16 ch5);
void unpackCompressed6ch(uint8_t *buf, uint16 *v0, uint16 *v1, uint16 *v2, \
							uint16 *v3, uint16 *v4, uint16 *v5);
void compress6chTestCodeBlocking(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//I�C Addresses - 7bits convention
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

//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_STRAINGAUGE_H
