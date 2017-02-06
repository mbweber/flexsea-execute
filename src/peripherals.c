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
	[This file] peripherals: code for the general peripheral modules
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//Note:  many init_x() functions are included in other files to keep a logical
// organization. Place them here if they are not specific to one file.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "peripherals.h"
#include "../../flexsea-system/inc/flexsea_system.h"
#include "ext_input.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8 uart_dma_rx_buf[96];	//ToDo #define
uint8 uart_dma_rx_buf_unwrapped[96];
uint8 uart_dma_tx_buf[96];
uint8 DMA_4_Chan;
uint8 DMA_4_TD[1];
uint8 gui_fsm_flag = DISABLED;

//int32 last_ang_read_period;
//int32 counts_since_last_ang_read;
int32 angtimer_read = 65000;//, last_angtimer_read = 65000;

//****************************************************************************
// Function(s)
//****************************************************************************

void init_exec_structures();

//Initialize and enables all the peripherals
void init_peripherals(void)
{
	//Motor control variables & peripherals:
	init_motor();	
    
	//Init Control:
	init_ctrl_data_structure();
	
	//Timebases:
	init_tb_timers();
    
	#if(MOTOR_COMMUT == COMMUT_SINE) 
    //Angle read timer
    init_angle_timer();
	#endif //(MOTOR_COMMUT == COMMUT_SINE) 

	//UART 2 - RS-485
	init_rs485();
	
	//UART 1 - Bluetooth
	init_bluetooth();
	
	//Analog, expansion port:
	init_analog();
	
	//Clutch:
	init_pwro();
	
	init_exec_structures();

	//Hall sensor for commutation?
	#if(ENC_COMMUT == ENC_HALL)
		Use_Hall_Write(HALL_PHYSICAL);	//Use Hall sensors (Expansion connector)
	#else
		#if (MOTOR_COMMUT == COMMUT_BLOCK)
		Use_Hall_Write(HALL_VIRTUAL);	//Do not use, or use software version
		#endif
	#endif
	
	//Enable Global Interrupts
    CyGlobalIntEnable; 
	
	//I2C0 - 3V3, IMU & Expansion
	#ifdef USE_I2C_0	
		
		init_i2c_0();
		
		//MPU-6500 IMU:
		#ifdef USE_IMU

		init_imu();
		CyDelay(25);
		init_imu();
		CyDelay(25);
		init_imu();
		CyDelay(25);
		
		#endif	//USE_IMU
		
		//External RGB LED:
		#ifdef USE_MINM_RGB
			
		//Set RGB LED - Starts Green
		i2c_init_minm(MINM_GREEN);
		
		#endif 	//USE_MINM_RGB
	
	#endif	//USE_I2C_0
	
	//I2C1 - 5V, Safety-CoP & strain gauge pot
	#ifdef USE_I2C_1	
		
		//I2C1 peripheral:
		init_i2c_1();
		
		//Strain amplifier:
		#ifdef USE_STRAIN
			
		init_strain();
		
		#endif	//USE_STRAIN
		
	#endif	//USE_I2C_1	
	
	//Magnetic encoder:
	#ifdef USE_AS5047		
	
	init_as5047();
	
	#endif //USE_AS5047
	
	//Die temperatuire measurement
	#ifdef USE_DIETEMP	
	DieTemp_1_GetTemp(&temp);
	#endif
	
	//Special color when waiting for USB (Yellow):
	set_led_rgb(1, 1, 0);
	
	//USB CDC
	#ifdef USE_USB	
	init_usb();
	#endif	//USE_USB
	
	//Notify the GUI that a FSM is running:
	#if(RUNTIME_FSM == ENABLED)
	gui_fsm_flag = ENABLED;
	#endif	//(RUNTIME_FSM == ENABLED)
	
	#if(MOTOR_COMMUT == COMMUT_SINE) 
	//Start converting:
	ADC_SAR_2_StartConvert();
	
	//Initialize structures:
	init_as504x(&as5047,10000); //10 is the sampling frequency in Hz
	init_as504x(&as5048b,250); //1 is the sampling frequency in Hz
	
	#endif	//(MOTOR_COMMUT == COMMUT_SINE) 
}

//Timebase timers init:
void init_tb_timers(void)
{
	//Timer 1: 1ms (LEDs, PID)
	Timer_1_Init();
	Timer_1_Start();
	isr_t1_Start();
}

void init_angle_timer(void)
{
    Timer_angleread_Start();
}

void init_exec_structures()
{
	exec1.enc_ang = &(as5047.signed_ang);
	exec1.enc_ang_vel = &(as5047.signed_ang_vel);
}

//update the number of counts since the last time the angle sensor was read
void update_counts_since_last_ang_read(struct as504x_s *as504x)
{
    angtimer_read = Timer_angleread_ReadCounter(); 
    
    if (angtimer_read>as504x->last_angtimer_read+20000)
    {
        as504x->counts_since_last_ang_read =  as504x->counts_since_last_ang_read + as504x->last_angtimer_read+65000-angtimer_read;
    }
    else
    {
        as504x->counts_since_last_ang_read = as504x->counts_since_last_ang_read + as504x->last_angtimer_read-angtimer_read;
    }
    as504x->last_angtimer_read = angtimer_read;
}

//resent the angle read counter and store the the counts between last two readings
void reset_ang_counter(struct as504x_s *as504x)
{
    update_counts_since_last_ang_read(as504x);
    as504x->last_ang_read_period = as504x->counts_since_last_ang_read-0;
    as504x->counts_since_last_ang_read = 0;
}

//Initialize encoder structures
void init_as504x(struct as504x_s *as504x, int sf)
{
	init_angsense(&as504x->raw);
    init_angsense(&as504x->filt);
    

    as504x->samplefreq = sf;
    as504x->last_angtimer_read = 0;
    as504x->counts_since_last_ang_read = 0;
    as504x->last_ang_read_period = 0;
    as504x->ang_abs_clks = 0; 
    as504x->ang_comp_clks = 0;
    as504x->num_rot = 0; 
}

void init_angsense(struct angsense_s *as)
{
    int i;
    for(i = 0; i < 11; i++)
		as->angs_clks[i] = 0;
        
    as->vels_cpms[0] = 0;
    as->vels_cpms[1] = 0;
    as->vels_ctrl_cpms[0] = 0;
    as->vels_ctrl_cpms[1] = 0;
    
    as->ang_clks = 0;
    as->ang_deg = 0;
    as->vel_cpms = 0;
    as->vel_ctrl_cpms = 0;
    as ->vel_rpm = 0;
}

int error_flag = 0;
//update all of the angle variables
void update_as504x(int32_t ang, struct as504x_s *as504x)
{  
    static int64_t raw_ang, raw_vel, raw_ctrl_vel;
    
    //determine if the encoder has rotated past 0/16383 point and in what direction
    if (ang-as504x->ang_abs_clks<-5000)
    {
        as504x->num_rot++;
    }
    else if ((ang-as504x->ang_abs_clks)>5000)
    {
        as504x->num_rot--;
    }
    
    //assign latest raw angle value
    as504x->ang_abs_clks = ang;
    
    //shift the ang values accept the last two, which will be shifted in the filter function
    int ii;
    for (ii=10;ii>1;ii--)
        as504x->raw.angs_clks[ii] = as504x->raw.angs_clks[ii-1];
        
    raw_ang = (as504x->num_rot<<14)+as504x->ang_abs_clks;    
    
    //calculate the simple difference velocity
    raw_vel = (as504x->raw.angs_clks[0]-as504x->raw.angs_clks[10]); //clicks per ms
    raw_ctrl_vel = as504x->raw.vels_cpms[0]*10;
    
   
    if (as504x->samplefreq == 10000)
    {
        as504x->filt.vel_cpms = filt_array_10khz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,20,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_10khz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_10khz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }
    else if (as504x->samplefreq == 250)
    {
        as504x->filt.vel_cpms = filt_array_250hz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,20,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_250hz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_250hz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }
    else if (as504x->samplefreq == 1000)
    {
        as504x->filt.vel_cpms = filt_array_1khz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,20,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_1khz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_1khz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }    
    
    //calculate the derived angular terms
    as504x->raw.ang_clks = as504x->raw.angs_clks[0];
    as504x->raw.ang_deg = (as504x->raw.ang_clks*360)>>14;
    as504x->raw.vel_cpms = as504x->raw.vels_cpms[0];  
    as504x->raw.vel_ctrl_cpms = as504x->raw.vel_cpms;
    as504x->raw.vel_rpm = (as504x->raw.vel_cpms*366)/100;
    as504x->filt.ang_deg = (as504x->filt.ang_clks*360)>>14;
    as504x->filt.vel_rpm = (as504x->filt.vel_cpms*366)/100;   

	as504x->signed_ang = -1 * raw_ang * MOTOR_ORIENTATION;
	as504x->signed_ang_vel = -1 * as504x->filt.vel_cpms * MOTOR_ORIENTATION;

    //update the 1 MHz counts from the last angle read
    update_counts_since_last_ang_read(as504x);
    
    //Update the velocity compensated angle
    as504x->ang_comp_clks = ((((as504x->counts_since_last_ang_read+90)*(as504x->filt.vel_cpms))/1000+(as504x->ang_abs_clks)+16384)%16384); 
}
