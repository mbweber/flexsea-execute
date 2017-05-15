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
	[This file] current_sensing: ADC configurations, read & filter functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-03-27 | jfduval | Released under GPL-3.0 release
****************************************************************************/

#ifndef INC_CURRENT_SENSING_H
#define INC_CURRENT_SENSING_H

//****************************************************************************
// Include(s)
//****************************************************************************	

#include "main.h"
//#include <flexsea_user_structs.h>

//****************************************************************************
// Prototype(s):
//****************************************************************************

void initCurrentSensing(void);
void adc_sar2_dma_config(void);
void update_current_arrays(void);
void set_current_zero(void);
void get_phase_currents(int32_t *);
void adc_sar2_dma_reinit(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//Motor current ADC:
#define ADC2_BUF_LEN				9
#define ADC2_BUF_LEN_3RD	        3

//DMA ADC SAR 2
#define DMA_1_BYTES_PER_BURST 		2
#define DMA_1_REQUEST_PER_BURST 	1
#define DMA_1_SRC_BASE 				(CYDEV_PERIPH_BASE)
#define DMA_1_DST_BASE 				(CYDEV_SRAM_BASE)

//****************************************************************************
// Shared variable(s)
//****************************************************************************	

extern int16 adc_dma_array[ADC2_BUF_LEN];
extern int16 adc_dma_array_buf[ADC2_BUF_LEN];
extern volatile uint8_t current_sensing_flag;
extern volatile int hallCurr;

//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_CURRENT_SENSING_H
