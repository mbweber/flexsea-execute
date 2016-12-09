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
int64_t motor_currents[2] = {0,0};
int64_t motor_currents_filt[2] = {0,0};
void current_rms_1(void)
{
	
    //the current measurements are separated into phases in order to assign the angle dependent motor constant to each phase
    //Current measurements -> 15.6 mAmps per IU
    //the line-to-line motor constant (what is usually given) needs to be divided by (3)^.5 for a sin wound motor and 2 for a trap wound motor
    //the current is divided by (3)^.5 so that a user can multiply the current by the common line-to-line motor constant
    calculating_current_flag = 1;
    static int phase_a_current, phase_b_current, phase_c_current;
    static int64_t raw_current;
    
    int phase_c_median = get_median(adc_dma_array_buf[0],adc_dma_array_buf[3],adc_dma_array_buf[6]);
    int phase_a_median = get_median(adc_dma_array_buf[1],adc_dma_array_buf[4],adc_dma_array_buf[7]);
    int phase_b_median = get_median(adc_dma_array_buf[2],adc_dma_array_buf[5],adc_dma_array_buf[8]);
    
    
    //the sum of current squareds that produces heat
    motortb.ex1[0] = (((phase_a_median-CURRENT_ZERO)*(phase_a_median-CURRENT_ZERO)+
                    (phase_b_median-CURRENT_ZERO)*(phase_b_median-CURRENT_ZERO)+
                    (phase_c_median-CURRENT_ZERO)*(phase_c_median-CURRENT_ZERO)))/41; //100 x Amps^2 = 15.6^2 * 100 / 10^6
  
    if (measure_motor_resistance)
    {
        phase_a_current = (int)((phase_a_median-CURRENT_ZERO))*16; //(15.6 mAmps/IU) / (955 sin amplitude) / 3^.5  = 0.0094 = 1/106
        phase_b_current = -(int)((phase_b_median-CURRENT_ZERO))*16;
        phase_c_current = -(int)((phase_c_median-CURRENT_ZERO))*16;
        raw_current = (phase_a_current+phase_b_current+phase_c_current);
    }
    else
    {
        phase_a_current = ((int)(((int)phaseAcoms[as5047.ang_comp_clks>>3])-955)*(phase_a_median-CURRENT_ZERO)); //(15.6 mAmps/IU) / (955 sin amplitude) / 3^.5  = 0.0094 = 1/106
        phase_b_current = ((int)(((int)phaseBcoms[as5047.ang_comp_clks>>3])-955)*(phase_b_median-CURRENT_ZERO)); //units should be mAmps
        phase_c_current = ((int)(((int)phaseCcoms[as5047.ang_comp_clks>>3])-955)*(phase_c_median-CURRENT_ZERO));
        raw_current = (phase_a_current+phase_b_current+phase_c_current)/106;   
    }
    calculating_current_flag = 0;
        
    //calculate the new filtered current 
    //the filter outputs raw values x 1024 in order to maintain precision
    ctrl.current.actual_val = PWM_SIGN*filt_array_10khz(motor_currents,motor_currents_filt,20,raw_current); // mAmps where I * the line-to-line motor constant = torque 
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

//1st order Butterworth LPF coefficiencts for cutoff frequencies from 1 to 50
int64_t as_10k[50] = {-65495,-65454,-65413,-65371,-65330,-65289,-65248,-65207,-65166,-65126,-65085,-65044,-65003,-64962,-64921,-64880,-64840,-64799,-64758,-64718,-64677,-64636,-64596,-64555,-64515,-64474,-64434,-64393,-64353,-64312,-64272,-64231,-64191,-64151,-64110,-64070,-64030,-63990,-63949,-63909,-63869,-63829,-63789,-63749,-63709,-63669,-63629,-63589,-63549,-63509};
int64_t bs_10k[50] = {21076,42139,63189,84226,105249,126259,147257,168240,189211,210169,231114,252045,272964,293870,314762,335642,356508,377362,398202,419030,439845,460647,481436,502212,522975,543726,564463,585188,605900,626599,647286,667959,688620,709269,729904,750527,771137,791735,812320,832892,853452,873999,894534,915056,935565,956062,976547,997019,1017478,1037925};
int64_t as_1k[50] = {-32563,-32359,-32156,-31955,-31754,-31555,-31358,-31161,-30965,-30771,-30578,-30386,-30195,-30005,-29817,-29629,-29442,-29257,-29072,-28889,-28706,-28525,-28345,-28165,-27987,-27809,-27632,-27457,-27282,-27108,-26935,-26763,-26592,-26422,-26252,-26084,-25916,-25749,-25583,-25417,-25253,-25089,-24926,-24764,-24603,-24442,-24282,-24123,-23965,-23807};
int64_t bs_1k[50] = {3284,6547,9791,13014,16218,19402,22567,25713,28840,31949,35039,38112,41166,44203,47223,50225,53210,56179,59131,62066,64985,67888,70776,73647,76504,79345,82171,84982,87778,90559,93326,96079,98818,101543,104254,106952,109636,112306,114964,117609,120240,122859,125466,128060,130641,133211,135769,138314,140848,143370};
int64_t as_250[50] = {-7989,-7790,-7597,-7407,-7222,-7041,-6864,-6691,-6521,-6354,-6191,-6031,-5874,-5719,-5567,-5418,-5271,-5127,-4985,-4845,-4707,-4571,-4437,-4305,-4174,-4045,-3918,-3792,-3668,-3545,-3423,-3303,-3184,-3066,-2949,-2834,-2719,-2605,-2492,-2380,-2269,-2158,-2049,-1940,-1831,-1723,-1616,-1509,-1403,-1297};
int64_t bs_250[50] = {813,1607,2382,3139,3879,4603,5311,6005,6684,7351,8004,8645,9274,9892,10499,11096,11683,12260,12829,13389,13941,14485,15021,15550,16072,16587,17096,17599,18096,18588,19074,19555,20032,20504,20971,21434,21893,22348,22800,23248,23693,24135,24574,25010,25443,25875,26304,26730,27155,27578};

//Filters raw signal at cut_off frequency if sampled at 10 kHz
//filt is 1024 x raw in order to maintain precision
//this function also shifts the arrays
int32_t filt_array_10khz(int64_t * raw, int64_t * filt, int cut_off, int64_t new_raw)
{  
    
    filt[1] = filt[0];
    raw[1] = raw[0];
    raw[0] = new_raw;
    //ensure the cut-off frequency is inbetween 1 and 50 Hz
    if (cut_off<1)
    cut_off = 1;
    else if (cut_off>50)
    cut_off = 50;
    
    filt[0] = (bs_10k[cut_off-1]*(raw[1]+raw[0])-as_10k[cut_off-1]*filt[1])>>16;   
    return (int32_t)(filt[0]>>10);
}

//Filters raw signal at cut_off frequency if sampled at 1 kHz
//filt is 32 x raw in order to maintain precision
//returns the filtered value at the correct scaling
int32_t filt_array_1khz(int64_t * raw, int64_t * filt, int cut_off, int64_t new_raw)
{  
    filt[1] = filt[0];
    raw[1] = raw[0];
    raw[0] = new_raw;
    //ensure the cut-off frequency is inbetween 1 and 50 Hz
    if (cut_off<1)
    cut_off = 1;
    else if (cut_off>50)
    cut_off = 50;
    
    filt[0] = (bs_1k[cut_off-1]*(raw[1]+raw[0])-as_1k[cut_off-1]*filt[1])>>15;   
    return (int32_t)(filt[0]>>5);
}

//Filters raw signal at cut_off frequency if sampled at 250 Hz
//filt is 8 x raw in order to maintain precision
//returns the filtered value at the correct scaling
int32_t filt_array_250hz(int64_t * raw, int64_t * filt, int cut_off, int64_t new_raw)
{  
    filt[1] = filt[0];
    raw[1] = raw[0];
    raw[0] = new_raw;
    //ensure the cut-off frequency is inbetween 1 and 50 Hz
    if (cut_off<1)
    cut_off = 1;
    else if (cut_off>50)
    cut_off = 50;
    
    filt[0] = (bs_250[cut_off-1]*(raw[1]+raw[0])-as_250[cut_off-1]*filt[1])>>13;   
    return (int32_t)(filt[0]>>3);
}

int get_median(int a, int b, int c)
{
    if ((a>=b && a<=c) || (a>=c && a<=b))
    return a;
    else if ((b>=a && b<=c) || (b>=c && b<=a))
    return b;
    
    return c;
}
