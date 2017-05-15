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
	[This file] analog: ADC configurations, read & filter functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	* 2017-02-06 | jfduval | Modified to support the 2 current sensing
	* 						 strategies (default & legacy)
****************************************************************************/

//Note: this is for the analog functionality of the expansion connector
// Current sensing (most of it) and strain gauge amplification are in 
// other files.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "analog.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

volatile uint16 adc1_res[ADC1_CHANNELS][ADC1_BUF_LEN];
volatile uint16 adc1_dbuf[ADC1_CHANNELS][ADC1_BUF_LEN];
volatile uint16 adc1_res_filtered[ADC1_CHANNELS];
uint16 adc_sar1_dma_array[ADC1_BUF_LEN + 1];

//DMA ADC SAR 1
uint8_t DMA_5_Chan;
uint8_t DMA_5_TD[1];

//****************************************************************************
// Function(s)
//****************************************************************************

void init_analog(void)
{
	//Analog amplifiers & multiplexer(s):
	AMux_1_Start();				//Start the MUX
	PGA_1_Start();
	PGA_2_Start();
	
	//ADC1:
	ADC_SAR_1_Start();
	adc_sar1_dma_config();
	isr_sar1_dma_Start();
	//ADC_SAR_1_StartConvert();	//Start converting
}

uint16 adc_avg8(uint16 new_data)
{
	uint32 sum = 0;
	static uint16 adc_avg_buf[8] = {0,0,0,0,0,0,0,0};
	static uint8_t cnt = 0;
	uint16 avg = 0;
	
	//Shift buffer and sum 7/8 terms
	for(cnt = 1; cnt < 8; cnt++)
	{
		adc_avg_buf[cnt-1] = adc_avg_buf[cnt];
		sum += adc_avg_buf[cnt-1];
	}
	adc_avg_buf[7] = new_data;
	sum += new_data;
		
	//Average
	avg = (uint16)(sum >> 3);
	
	return avg;	
}

//Filters the ADC buffer
void filter_sar_adc(void)
{
	uint16_t i = 0, j = 0;
	uint32 adc_sum = 0;
	
	//For each channel:
	for(i = 0; i < ADC1_CHANNELS; i++)
	{
		//For each value in the channel:
		adc_sum = 0;
		for(j = 0; j < ADC1_BUF_LEN; j++)
		{
			//Add the values
			adc_sum += (uint32)adc1_res[i][j];
		}
		
		//And divide to get mean
		adc1_res_filtered[i] = (uint16) ((uint32)adc_sum >> ADC1_SHIFT);
	}
}

//To avoid data corruption we copy the buffer during the interrupt:
void double_buffer_adc(void)
{
	uint16 i = 0, j = 0;
	
	//For each channel:
	for(i = 0; i < ADC1_CHANNELS; i++)
	{
		//For each value in the channel:
		for(j = 0; j < ADC1_BUF_LEN; j++)
		{	
			adc1_dbuf[i][j] = adc1_res[i][j];
		}
	}
}

//Returns one filtered value
int16 read_analog(uint8_t ch)
{
	if(ch < ADC1_CHANNELS)
	{
		//Valid channel, return value
		return adc1_res_filtered[ch];
	}

	//Otherwise return 0
	return 0;
}

//DMA for ADC SAR 1 transfers (Expansion, VB_SNS, etc.)
//Triggers an ISR after N samples
void adc_sar1_dma_config(void)
{
	DMA_5_Chan = DMA_5_DmaInitialize(DMA_5_BYTES_PER_BURST, DMA_5_REQUEST_PER_BURST,
	    HI16(DMA_5_SRC_BASE), HI16(DMA_5_DST_BASE));
	DMA_5_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_5_TD[0], ADC1_BUF_LEN*2, DMA_5_TD[0], DMA_5__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(DMA_5_TD[0], LO16((uint32)ADC_SAR_1_SAR_WRK0_PTR), LO16((uint32)adc_sar1_dma_array));
	CyDmaChSetInitialTd(DMA_5_Chan, DMA_5_TD[0]);
	CyDmaChEnable(DMA_5_Chan, 1);
}
