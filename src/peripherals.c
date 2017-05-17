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
#include "motor.h"
#include "control.h"
#include "analog.h"
#include "serial.h"
#include "ext_output.h"
#include "../../flexsea-system/inc/flexsea_system.h"
#include "ext_input.h"
#include "user-ex.h"
#include "i2c.h"
#include "imu.h"
#include "strain.h"
#include "ui.h"
#include "usb.h"
#include "mag_encoders.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t gui_fsm_flag = DISABLED;

//****************************************************************************
// Function(s)
//****************************************************************************

//Initialize and enables all the peripherals
void init_peripherals(void)
{
    //Magnetic encoder:
	#ifdef USE_AS5047			
	init_as5047();
	#endif //USE_AS5047
    
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
	
		//Initialize structures:
		init_as504x(&as5047);
	
	#endif	//(MOTOR_COMMUT == COMMUT_SINE) 
}

//Timebase timers init:
void init_tb_timers(void)
{
	//Timer 1: 1ms (LEDs, PID)
	Timer_1_Init();
	Timer_1_Start();
	Timer_1_WritePeriod(4000);	//10kHz
	isr_t1_Start();
}
