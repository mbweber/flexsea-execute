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

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "current_sensing.h"
#include "control.h"
#include "sensor_commut.h"
#include "mag_encoders.h"
#include "filters.h"
#include "user-ex.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

volatile int hallCurr = 0;
int calculating_current_flag = 0;

int16 adc_dma_array[ADC2_BUF_LEN];
int16 adc_dma_array_buf[ADC2_BUF_LEN];

volatile uint8_t current_sensing_flag = 0;

int currPhase[8][3] = {{0,0,0}, {1,-1,0}, {-1,0,1}, {0,-1,1}, \
                       {0,1,-1}, {1,0,-1}, {-1,1,0}, {0,0,0}};

int32_t phase_a_zero = 0;
int32_t phase_b_zero = 0;
int32_t phase_c_zero = 0;

//DMA ADC SAR 2
uint8_t DMA_1_Chan;
uint8_t DMA_1_TD[1];

int64_t motor_currents[2] = {0,0};
int64_t motor_currents_filt[2] = {0,0};

//****************************************************************************
// Function(s)
//****************************************************************************

void initCurrentSensing(void)
{
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	
		//ADC2: Motor current
		ADC_SAR_2_Start();
		adc_sar2_dma_config();
		isr_sar2_dma_Start();
		
		#if(CURRENT_SENSING != CS_LEGACY)
			
			ADC_SAR_2_IRQ_Enable();
		
		#endif
	
	#else	//(MOTOR_COMMUT == COMMUT_BLOCK)
	
	    //VDAC8: OpAmp VREF
		VDAC8_1_Start();
		
		//Analog amplifiers & multiplexer(s):
		Opamp_1_Start();    
	        
	    //ADC2: Motor current
		ADC_SAR_2_Start();	
		ADC_SAR_2_IRQ_Enable();
		
		CyDelay(1);
		
		adc_sar2_dma_config();
		isr_sar2_dma_Start();    
	    CyDelay(1);
	
	#endif
}

//DMA for ADC SAR 2 transfers (motor current sensing)
//Triggers an ISR after X transfers
void adc_sar2_dma_config(void)
{
	uint8_t xferLen = 0;
	
	#if((MOTOR_COMMUT == COMMUT_BLOCK) && (CURRENT_SENSING == CS_LEGACY))

		//5 transfers per ISR (10 bytes):	
		xferLen = 10;	
	
	#else
	
		//3 transfers per ISR (6 bytes):	
		xferLen = 6;	//3 samples
	
	#endif
	
	//X transfers per ISR (2X bytes):	
	DMA_1_Chan = DMA_1_DmaInitialize(DMA_1_BYTES_PER_BURST, DMA_1_REQUEST_PER_BURST, 
	    HI16(DMA_1_SRC_BASE), HI16(DMA_1_DST_BASE));
	DMA_1_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_1_TD[0], xferLen, DMA_1_TD[0], DMA_1__TD_TERMOUT_EN | TD_INC_DST_ADR);

	CyDmaTdSetAddress(DMA_1_TD[0], LO16((uint32)ADC_SAR_2_SAR_WRK0_PTR), LO16((uint32)adc_dma_array));
	CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
	CyDmaChEnable(DMA_1_Chan, 1);
}

void update_current_arrays(void)
{
	adc_dma_array_buf[0] = adc_dma_array[0];
	adc_dma_array_buf[1] = adc_dma_array[1];
    adc_dma_array_buf[2] = adc_dma_array[2];       

	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	static int phase_a_current, phase_b_current, phase_c_current;
	int phase_c_median = get_median(adc_dma_array_buf[0],adc_dma_array_buf[3],adc_dma_array_buf[6]);
    int phase_a_median = get_median(adc_dma_array_buf[1],adc_dma_array_buf[4],adc_dma_array_buf[7]);
    int phase_b_median = get_median(adc_dma_array_buf[2],adc_dma_array_buf[5],adc_dma_array_buf[8]);
	#endif
    
    //the current measurements are separated into phases in order to assign the angle dependent motor constant to each phase
    //Current measurements -> 16.16 mAmps per IU
    //the line-to-line motor constant (what is usually given) needs to be divided by (3)^.5 for a sin wound motor and 2 for a trap wound motor
    //the current is divided by (3)^.5 so that a user can multiply the current by the common line-to-line motor constant

    static int64_t raw_current;
    static int32 phase_a_raw, phase_b_raw, phase_c_raw; 
    static int32_t phase_a_ang, phase_b_ang, phase_c_ang; 
    static int32_t phase_a_com, phase_b_com, phase_c_com;
    
    phase_b_raw = (adc_dma_array_buf[2]-phase_b_zero);  
    phase_c_raw = (adc_dma_array_buf[0]-phase_c_zero);
    phase_a_raw = (adc_dma_array_buf[1]-phase_a_zero);     
    
    if (measure_motor_resistance)
    {
        raw_current = ((int)(phase_a_raw))-((int)(phase_b_raw))-((int)(phase_c_raw));
    }
    else
    {
		#if(MOTOR_COMMUT == COMMUT_SINE)
            phase_a_ang = ((((-60+user_data_1.w[2])*(as5047.filt_vel_cpms))/1000+(as5047.ang_abs_clks)+16384)%16384);
            phase_b_ang = ((((-10+user_data_1.w[2])*(as5047.filt_vel_cpms))/1000+(as5047.ang_abs_clks)+16384)%16384);
            phase_c_ang = ((((-110+user_data_1.w[2])*(as5047.filt_vel_cpms))/1000+(as5047.ang_abs_clks)+16384)%16384);
            
            phase_a_com = (int)phaseAcoms[phase_a_ang>>3];
            phase_b_com = (int)phaseBcoms[phase_b_ang>>3];
            phase_c_com = (int)phaseCcoms[phase_c_ang>>3];
            
            static int32_t cursum,cursomcntr;
            cursum = 0;
            cursomcntr = 0;
            
            if (phase_a_com>240 || phase_a_com<-240)
            {
                cursum += (phase_a_raw*495)/phase_a_com;
                cursomcntr++;
            }
            if (phase_b_com>240 || phase_b_com<-240)
            {
                cursum += (phase_b_raw*495)/phase_b_com;
                cursomcntr++;
            }
            if (phase_c_com>240 || phase_c_com<-240)
            {
                cursum += (phase_c_raw*495)/phase_c_com;
                cursomcntr++;
            }
            
            if (cursomcntr>0)
            {
                raw_current = -(24*cursum)/cursomcntr;
            }
            
            //This is the amplitude of the current and should be multiplied by the phase torque constant or line-to-line constant / 3^.5
             //x 16 mA/IU /2 samples * 3/2 for sum of 3 sin^2        
		#endif
		
		#if((MOTOR_COMMUT == COMMUT_BLOCK) && (CURRENT_SENSING != CS_LEGACY))
			
			hallCurr = Status_Reg_1_Read();
			phase_a_current = currPhase[hallCurr][2]*(phase_a_median-CURRENT_ZERO);
	        phase_b_current = currPhase[hallCurr][1]*(phase_b_median-CURRENT_ZERO);
	        phase_c_current = currPhase[hallCurr][0]*(phase_c_median-CURRENT_ZERO);
			
			//Sign based on motor direction
			if(MotorDirection_Control == 0)
			{	
				raw_current = -9*(phase_a_current+phase_b_current+phase_c_current);
			}
			else
			{
				raw_current = 9*(phase_a_current+phase_b_current+phase_c_current);
			}
	        
			
		#endif
    }
    
    //calculate the new filtered current
    //the filter outputs raw values x 1024 in order to maintain precision
    //ctrl.current.actual_val = MOTOR_ORIENTATION*filt_array_10khz(motor_currents,motor_currents_filt,40,raw_current); // mAmps where I * the line-to-line motor constant = torque 
    ctrl.current.actual_val = MOTOR_ORIENTATION*raw_current; // mAmps where I * the line-to-line motor constant = torque 
    update_diffarr(&ctrl.current.actual_vals,ctrl.current.actual_val,50);    
}

void set_current_zero()
{
    static int32_t ii =0;
    static int32_t a_sum = 0, b_sum = 0, c_sum = 0;
    
    ii++;
    b_sum += (int32_t)adc_dma_array_buf[2];
    c_sum += (int32_t)adc_dma_array_buf[0];
    a_sum += (int32_t)adc_dma_array_buf[1];    
    
    phase_a_zero = (a_sum+ii/2)/ii;
    phase_b_zero = (b_sum+ii/2)/ii;
    phase_c_zero = (c_sum+ii/2)/ii;    
}

void get_phase_currents(int32_t * phase_curs)
{       
    phase_curs[1] = (adc_dma_array_buf[2]-phase_b_zero)*16;   
    phase_curs[2] = (adc_dma_array_buf[0]-phase_c_zero)*16;
    phase_curs[0] = (adc_dma_array_buf[1]-phase_a_zero)*16;   
}

void adc_sar2_dma_reinit(void)
{
CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
}
