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
	[This file] analog: ADC configurations, read & filter functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//Note: this is for the analog functionality of the expansion connector
// Current sensing and strain gauge amplification are in other files.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "analog.h"
#include <stdint.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

volatile uint16 adc1_res[ADC1_CHANNELS][ADC1_BUF_LEN];
volatile uint16 adc1_dbuf[ADC1_CHANNELS][ADC1_BUF_LEN];
volatile uint16 adc1_res_filtered[ADC1_CHANNELS];

int32 phase_a_currents[ADC2_BUF_LEN_3RD];
int32 phase_b_currents[ADC2_BUF_LEN_3RD];
int32 phase_c_currents[ADC2_BUF_LEN_3RD];
int calculating_current_flag = 0;

int16 adc_dma_array[ADC2_BUF_LEN];
int16 adc_dma_array_buf[ADC2_BUF_LEN];
uint16 adc_sar1_dma_array[ADC1_BUF_LEN + 1];
volatile uint8 amux_ch = 0;

volatile uint8 current_sensing_flag = 0;

//DMA ADC SAR 1
uint8 DMA_5_Chan;
uint8 DMA_5_TD[1];

//DMA ADC SAR 2 (Current sensing)
uint8 DMA_1_Chan;
uint8 DMA_1_TD[1];

//****************************************************************************
// Function(s)
//****************************************************************************

uint32_t isqrt_impl(uint32_t const n, uint32_t const xk)
{
    uint32_t const xk1 = (xk + n / xk) / 2;
    return (xk1 >= xk) ? xk : isqrt_impl(n, xk1);
}

uint32_t isqrt(uint64_t const n)
{
    if (n == 0) return 0;
    if (n == 18446744073709551615ULL) return 4294967295U;
    return isqrt_impl(n, n);
}

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
	ADC_SAR_1_StartConvert();	//Start converting
}

uint16 adc_avg8(uint16 new_data)
{
	uint32 sum = 0;
	static uint16 adc_avg_buf[8] = {0,0,0,0,0,0,0,0};
	static uint8 cnt = 0;
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
	uint8 i = 0, j = 0;
	uint32 adc_sum = 0;
	
	//For each channel:
	for(i = 0; i < ADC1_CHANNELS; i++)
	{
		//For each value in the channel:
		adc_sum = 0;
		for(j = 0; j < ADC1_BUF_LEN; j++)
		{
			//Add the values
			adc_sum += (uint32)adc1_dbuf[i][j];
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
int16 read_analog(uint8 ch)
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
//Triggers an ISR after 9 samples
void adc_sar1_dma_config(void)
{
	DMA_5_Chan = DMA_5_DmaInitialize(DMA_5_BYTES_PER_BURST, DMA_5_REQUEST_PER_BURST,
	    HI16(DMA_5_SRC_BASE), HI16(DMA_5_DST_BASE));
	DMA_5_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_5_TD[0], 18, DMA_5_TD[0], DMA_5__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(DMA_5_TD[0], LO16((uint32)ADC_SAR_1_SAR_WRK0_PTR), LO16((uint32)adc_sar1_dma_array));
	CyDmaChSetInitialTd(DMA_5_Chan, DMA_5_TD[0]);
	CyDmaChEnable(DMA_5_Chan, 1);
}

//DMA for ADC SAR 2 transfers (motor current sensing)
//Triggers an ISR after X transfers
void adc_sar2_dma_config(void)
{
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
		
	//5 transfers per ISR (10 bytes):	
	DMA_1_Chan = DMA_1_DmaInitialize(DMA_1_BYTES_PER_BURST, DMA_1_REQUEST_PER_BURST, 
	    HI16(DMA_1_SRC_BASE), HI16(DMA_1_DST_BASE));
	DMA_1_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_1_TD[0], 10, DMA_1_TD[0], DMA_1__TD_TERMOUT_EN | TD_INC_DST_ADR);
	
	#else
	
	//9 transfers per ISR (18 bytes):	
	DMA_1_Chan = DMA_1_DmaInitialize(DMA_1_BYTES_PER_BURST, DMA_1_REQUEST_PER_BURST, 
		HI16(DMA_1_SRC_BASE), HI16(DMA_1_DST_BASE));
	DMA_1_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_1_TD[0], ADC2_BUF_LEN*2, DMA_1_TD[0], DMA_1__TD_TERMOUT_EN | TD_INC_DST_ADR);
	
	#endif

	CyDmaTdSetAddress(DMA_1_TD[0], LO16((uint32)ADC_SAR_2_SAR_WRK0_PTR), LO16((uint32)adc_dma_array));
	CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
	CyDmaChEnable(DMA_1_Chan, 1);
}

//Calculates the motor current
int motor_currents[2] = {0,0};
int motor_currents_filt[2] = {0,0};
void current_rms_1(void)
{
	
    //the current measurements are separated into phases in order to assign the angle dependent motor constant to each phase
    //Current measurements -> 15.6 mAmps per IU
    //the line-to-line motor constant (what is usually given) needs to be divided by (3)^.5 for a sin wound motor and 2 for a trap wound motor
    //the current is divided by (3)^.5 so that a user can multiply the current by the common line-to-line motor constant
    calculating_current_flag = 1;
    
    
    int phase_c_median = get_median(adc_dma_array_buf[0],adc_dma_array_buf[3],adc_dma_array_buf[6]);
    int phase_a_median = get_median(adc_dma_array_buf[1],adc_dma_array_buf[4],adc_dma_array_buf[7]);
    int phase_b_median = get_median(adc_dma_array_buf[2],adc_dma_array_buf[5],adc_dma_array_buf[8]);
    
    int phase_a_current = ((int)(((int)phaseAcoms[as5047.angle_comp>>3])-955)*(phase_a_median-CURRENT_ZERO))/106; //(15.6 mAmps/IU) / (955 sin amplitude) / 3^.5  = 0.0094 = 1/106
    int phase_b_current = ((int)(((int)phaseBcoms[as5047.angle_comp>>3])-955)*(phase_b_median-CURRENT_ZERO))/106; //units should be mAmps
    int phase_c_current = ((int)(((int)phaseCcoms[as5047.angle_comp>>3])-955)*(phase_c_median-CURRENT_ZERO))/106;
    
    calculating_current_flag = 0;


    //shift raw and filtered current arrays
    motor_currents[1] = motor_currents[0];
    motor_currents_filt[1] = motor_currents_filt[0]; 
    motor_currents[0] = (phase_a_current+phase_b_current+phase_c_current)>>5;
    
    //calculate the new filtered current 
    //the filter outputs raw values x 1024 in order to maintain precision
    filt_array(motor_currents,motor_currents_filt);

    //current is divided by 1024 to account for filtering
    ctrl.current.actual_val = (int32)(PWM_SIGN*motor_currents_filt[0]>>5); // mAmps where I * the line-to-line motor constant = torque
    

    //global_variable_1 = (adc_dma_array_buf[0]-CURRENT_ZERO)+(adc_dma_array_buf[1]-CURRENT_ZERO)+(adc_dma_array_buf[2]-CURRENT_ZERO);
    //global_variable_2 = (phase_a_currents[0]*phase_a_currents[0] + phase_b_currents[0]*phase_b_currents[0] + phase_c_currents[0]*phase_c_currents[0])/2924;
    //global_variable_3 = (ctrl.current.actual_val*ctrl.current.actual_val)/26316;
    

    
    

}

//update the current measurement buffer
void update_current_arrays(void)
{
    int ii;
    if (calculating_current_flag == 0)
    {
        for (ii = 0; ii<ADC2_BUF_LEN;ii++)
        {
            adc_dma_array_buf[ii] = adc_dma_array[ii];
        }
    }    
}


//Filters raw signal at 30 Hz cutoff if sampled at 10 kHz
//filt is 1024 x raw in order to maintain precision
void filt_array(int * raw,int * filt)
{
    filt[0] = (int)(((9791*(raw[0]+raw[1]))
              +1005*filt[1]+512)>>10);
}

int get_median(int a, int b, int c)
{
    if ((a>=b && a<=c) || (a>=c && a<=b))
    return a;
    else if ((b>=a && b<=c) || (b>=c && b<=a))
    return b;
    
    return c;
}
