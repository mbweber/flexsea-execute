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
	[This file] motor: motor control functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//Note: Any control function that's not open loop is in control.c/h
//		This file only has open loop controllers (PWM, pulse, etc)

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "motor.h"
#include "control.h"
#include "current_sensing.h"
#include "ext_input.h"
#include "safety.h"
#include "user-ex.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"
#include "mem_angle.h"
#include "sensor_commut.h"
#include "dynamic_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t hall_conv[6] = {5,4,6,2,3,1};
uint16_t myPWMcompareA = 0, myPWMcompareB = 0, myPWMcompareC = 0;

//****************************************************************************
// Function(s)
//****************************************************************************

//Initializes all the variable
void init_motor(void)
{
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
		
	//PWM1: BLDC - Block commutation
	PWM_1_Start();
	PWM_1_WriteCompare1(0);			//Start at 0%
	PWM_1_WriteCompare2(PWM2DC(0));	
	
	//Default is Brake mode:
	Coast_Brake_Write(1);
	
	initCurrentSensing();
	
	#if(CURRENT_SENSING != CS_LEGACY)
		
		ADC_SAR_2_IRQ_Enable();
	
	#endif
	
	#else	//(MOTOR_COMMUT == COMMUT_BLOCK)
	
    initCurrentSensing();  
        
	//Start 3 PWM at 0%  
	PWM_A_Start();
	//PWM_A_WriteCompare1(0);		//Edge: Compare1, Center: Compare
	PWM_A_WriteCompare(PWM_AMP);
    PWM_B_Start();
	//PWM_B_WriteCompare1(0);
	PWM_B_WriteCompare(PWM_AMP);
	PWM_C_Start();
	//PWM_C_WriteCompare1(0);
	PWM_C_WriteCompare(PWM_AMP);
	setDmaPwmCompare(PWM_AMP,PWM_AMP,PWM_AMP);
	initDmaPwmCompare();
	isr_mot_Start();
    
    Control_Reg_1_Write(1);
    CyDelay(1);     
	
	//Angle table can be stored in EEPROM or FLASH:
	#ifdef USE_EEPROM		
	init_eeprom();	
    load_eeprom_to_angles();
	#endif	//USE_EEPROM
	
	#ifdef USE_FLASH		
	init_flash();		
	#endif	//USE_FLASH
		
	#endif	//(MOTOR_COMMUT == COMMUT_BLOCK)

	//Quadrature 1: Motor shaft encoder
	#ifdef USE_QEI
	init_qei();
	#endif	//USE_QEI
	
	//When using Brushed, fixed Hall code:
	#if(MOTOR_TYPE == MOTOR_BRUSHED)
	Use_Hall_Write(HALL_VIRTUAL);
	Virtual_Hall_Write(0b110); 
	#endif	//MOTOR_TYPE == MOTOR_BRUSHED	
}

//TODO: rename to setMotorVoltage, or something like that
// takes as an argument the voltage to set the motor to, in milliVolts
// applies a PWM to the motor, considering the nonlinear relationship between PWM and motor voltage
// also accounts for the linear relationship between motor voltage & battery voltage
void motor_open_speed_1(int32 voltageToApply)
{
	const int32_t MAX_VOLTAGE = 50000;
	const int32_t MAX_PWM_ALLOWED = 1024;
	
	//store voltage in an int32, we are gonna save resolution by magnifying intermediate values by like 1000
	int32_t v = (int32_t)voltageToApply;
	
	//impose a max magnitude on user input voltage
	v = (v > MAX_VOLTAGE) ? MAX_VOLTAGE : v;
	v = (v < -1*MAX_VOLTAGE) ? -1*MAX_VOLTAGE : v;
	
	int32_t pwmToApply = 0;

    
	//only set a pwm if we have a legal/valid battery voltage
	if(17000 <  safety_cop.v_vb_mv &&  safety_cop.v_vb_mv < 54000)
	{	
		pwmToApply = (v*76) / ( safety_cop.v_vb_mv>>4);
	}
	
    #if (MOTOR_COMMUT == COMMUT_BLOCK)
		
		uint16 tmp = 0;
    
         //Clip PWM to valid range
    	if(pwm_duty >= MAX_PWM)
    		pdc = MAX_PWM;
    	else if(pwm_duty <= MIN_PWM)
    		pdc = MIN_PWM;
    	else
    		pdc = pwm_duty;
    	
    	//User defined sign:
    	pdc = pdc * PWM_SIGN;
    	
    	//Save value to structure:
    	ctrl.pwm = pdc;
    	
    	//Change direction according to sign
    	if(pdc < 0)
    	{
    		pdc = -pdc;	//Make it positive
    		MotorDirection_Write(0);

    	}
    	else
    	{
    		MotorDirection_Write(1);
    	}
    	
    	//Write duty cycle to PWM module
    	tmp = PWM1DC((uint16)pdc);
    	PWM_1_WriteCompare1(tmp);
    	PWM_1_WriteCompare2(PWM2DC(tmp));	//Can't be 0 or the ADC won't trigger
        
    #else
		
        if(pwmToApply >= MAX_PWM_ALLOWED)
    		pwmToApply = MAX_PWM_ALLOWED;
    	else if(pwmToApply <= -1*MAX_PWM_ALLOWED)
    		pwmToApply = -1*MAX_PWM_ALLOWED;
        
        exec1.sine_commut_pwm = MOTOR_ORIENTATION*(int16_t)(pwmToApply);
    #endif
    
    //dynamicUserData.pwm = MOTOR_ORIENTATION*(int16_t)(pwmToApply);
    //dynamicUserData.mot_vol = v/10;
}

//Controls motor PWM duty cycle
//Manual sign (-1 or 1, everything else will set the PWM at 0)
void motor_open_speed_2(int16 pwm_duty, int sign)
{
	int16 pdc = 0;
	
	//Clip PWM to valid range (0 - Maximum)
	if(pwm_duty >= MAX_PWM)
		pdc = MAX_PWM;
	else if(pwm_duty <= 0)
		pdc = 0;
	else
		pdc = pwm_duty;
	
	//User defined sign:
	sign = sign * MOTOR_ORIENTATION;
	
	//Change direction according to sign
	if(sign == -1)
	{
		#if (MOTOR_COMMUT == COMMUT_BLOCK)
		MotorDirection_Write(0);
		#else
		//ToDo Sine
		#endif
	}
	else if(sign == 1)
	{
		#if (MOTOR_COMMUT == COMMUT_BLOCK)
		MotorDirection_Write(1);
		#else
		//ToDo Sine
		#endif
	}
	else
	{
		//Invalid sign, forces PWM to 0
		pdc = 0;
	}
	
	//Write duty cycle to PWM module
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	pdc = PWM1DC(pdc);
	PWM_1_WriteCompare1(pdc);
	PWM_1_WriteCompare2(PWM2DC(pdc));	//Can't be 0 or the ADC won't trigger
	#else
	exec1.sine_commut_pwm = pdc;
	#endif
}

//PWM Compare1 registers are set via DMA, not via the API function calls:
void initDmaPwmCompare(void)
{
	uint8 DMA_PA_Chan, DMA_PB_Chan, DMA_PC_Chan;
	uint8 DMA_PA_TD[1], DMA_PB_TD[1], DMA_PC_TD[1];

	//DMA Configuration for DMA_PA:
	DMA_PA_Chan = DMA_PA_DmaInitialize(DMA_PX_BYTES_PER_BURST, DMA_PX_REQUEST_PER_BURST, 
	    HI16(DMA_PX_SRC_BASE), HI16(DMA_PX_DST_BASE));
	DMA_PA_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_PA_TD[0], 2, DMA_PA_TD[0], DMA_PA__TD_TERMOUT_EN);
	CyDmaTdSetAddress(DMA_PA_TD[0], LO16((uint32)&myPWMcompareA), LO16((uint32)PWM_A_COMPARE1_LSB_PTR));
	CyDmaChSetInitialTd(DMA_PA_Chan, DMA_PA_TD[0]);
	CyDmaChEnable(DMA_PA_Chan, 1);
	
	//DMA Configuration for DMA_PB:
	DMA_PB_Chan = DMA_PB_DmaInitialize(DMA_PX_BYTES_PER_BURST, DMA_PX_REQUEST_PER_BURST, 
	    HI16(DMA_PX_SRC_BASE), HI16(DMA_PX_DST_BASE));
	DMA_PB_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_PB_TD[0], 2, DMA_PB_TD[0], DMA_PB__TD_TERMOUT_EN);
	CyDmaTdSetAddress(DMA_PB_TD[0], LO16((uint32)&myPWMcompareB), LO16((uint32)PWM_B_COMPARE1_LSB_PTR));
	CyDmaChSetInitialTd(DMA_PB_Chan, DMA_PB_TD[0]);
	CyDmaChEnable(DMA_PB_Chan, 1);
	
	//DMA Configuration for DMA_PC:
	DMA_PC_Chan = DMA_PC_DmaInitialize(DMA_PX_BYTES_PER_BURST, DMA_PX_REQUEST_PER_BURST, 
	    HI16(DMA_PX_SRC_BASE), HI16(DMA_PX_DST_BASE));
	DMA_PC_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_PC_TD[0], 2, DMA_PC_TD[0], DMA_PC__TD_TERMOUT_EN);
	CyDmaTdSetAddress(DMA_PC_TD[0], LO16((uint32)&myPWMcompareC), LO16((uint32)PWM_C_COMPARE1_LSB_PTR));
	CyDmaChSetInitialTd(DMA_PC_Chan, DMA_PC_TD[0]);
	CyDmaChEnable(DMA_PC_Chan, 1);
}

//Use this to set the new PWM Compare1 values:
void setDmaPwmCompare(uint16_t a, uint16_t b, uint16_t c)
{
	myPWMcompareA = a;
	myPWMcompareB = b;
	myPWMcompareC = c;
}

//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

//Sends a constant PWM. Blocking.
void motor_fixed_pwm_test_code_blocking(int spd)
{
	uint8_t toggle_wdclk = 0;	
	
	ctrl.active_ctrl = CTRL_OPEN;	
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);	//Brake
	#endif
	motor_open_speed_1(spd);
	
	while(1)
	{	
		LED_R_Write(EX1_Read());
		LED_G_Write(EX2_Read());
		LED_B_Write(EX3_Read());
		
		//WatchDog Clock (Safety-CoP)
		toggle_wdclk ^= 1;
		WDCLK_Write(toggle_wdclk);
		
		refresh_enc_control();
	}
}

//Sends a constant PWM. Non-Blocking.
void motor_fixed_pwm_test_code_non_blocking(int spd)
{
	ctrl.active_ctrl = CTRL_OPEN;	
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);	//Brake
	#endif
	motor_open_speed_1(spd);	
}
