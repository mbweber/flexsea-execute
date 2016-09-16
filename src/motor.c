//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@mit.edu
// 02/2015
//****************************************************************************
// motor: motor control functions
//****************************************************************************

//Note: Any control function that's not open loop is in control.c/h
//		This file only has open loop controllers (PWM, pulse, etc)

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "motor.h"
#include "analog.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8 hall_conv[6] = {5,4,6,2,3,1};
int32 sine_commut_pwm = 0;

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
	
	#else	//(MOTOR_COMMUT == COMMUT_BLOCK)
		
	//Start 3 PWM at 0%  
	PWM_A_Start();
	PWM_A_WriteCompare1(0);			  
    PWM_B_Start();
	PWM_B_WriteCompare1(0);
	PWM_C_Start();
	PWM_C_WriteCompare1(0);
    
    Control_Reg_1_Write(1);
    //Control_Reg_2_Write(1);

	
	//Angle table can be stored in EEPROM or FLASH:
	#ifdef USE_EEPROM		
	init_eeprom();		
	#endif	//USE_EEPROM
	
	#ifdef USE_FLASH		
	init_flash();		
	#endif	//USE_FLASH
		
	#endif	//(MOTOR_COMMUT == COMMUT_BLOCK)

	//ADC2: Motor current
	ADC_SAR_2_Start();
	adc_sar2_dma_config();
	isr_sar2_dma_Start();
	ADC_SAR_2_IRQ_Enable();
    
	
	//VDAC8: OpAmp VREF
	VDAC8_1_Start();
	
	//Analog amplifiers & multiplexer(s):
	Opamp_1_Start();

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

//Controls motor PWM duty cycle
//Sign of 'pwm_duty' determines rotation direction
void motor_open_speed_1(int16 pwm_duty)
{
	int16 pdc = 0;
	uint16 tmp = 0;

    #if (MOTOR_COMMUT == COMMUT_BLOCK)
    
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
        if(pwm_duty >= 1024)
    		pdc = 1024;
    	else if(pwm_duty <= -1024)
    		pdc = -1024;
    	else
    		pdc = pwm_duty;
        
        pdc = pdc*PWM_SIGN;
        
        sine_commut_pwm = pdc;
    #endif
    
    
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
	sign = sign * PWM_SIGN;
	
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
	sine_commut_pwm = pdc;
	#endif
}

//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

//Sends a constant PWM. Blocking.
void motor_fixed_pwm_test_code_blocking(int spd)
{
	uint8 toggle_wdclk = 0;	
	
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

//Use this to send PWM pulses in open speed mode
void test_pwm_pulse_blocking(void)
{
	uint16 val = 0;
	
	ctrl.active_ctrl = CTRL_OPEN;
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);	//Brake
	#endif
	motor_open_speed_1(0);	
	
	while(1)
	{	
		//RGB LED = Hall code:
		LED_R_Write(EX1_Read());
		LED_G_Write(EX2_Read());
		LED_B_Write(EX3_Read());
		
		val = output_step();
		motor_open_speed_1(val);
	}
}

//Use before main while() as a basic test
void motor_stepper_test_blocking_1(int spd)
{
	uint8 hall_code_0 = 0, hall_code = 0;
	
	ctrl.active_ctrl = CTRL_OPEN;
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);	//Brake
	#endif
	motor_open_speed_1(spd);
	
	while(1)
	{
		hall_code_0++;
		hall_code_0 %= 6;
		hall_code = hall_conv[hall_code_0];
		
		#if (MOTOR_COMMUT == COMMUT_BLOCK)
		Virtual_Hall_Write(hall_code);
		#endif
		
		LED_R_Write(hall_code & 0x01);
		LED_G_Write((hall_code & 0x02)>>1);
		LED_B_Write((hall_code & 0x04)>>2);
		
		CyDelay(10);
	}
}

//To test with the full stack, use this init...
void motor_stepper_test_init(int spd)
{
	ctrl.active_ctrl = CTRL_OPEN;
	#if (MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);
	#endif
	motor_open_speed_1(spd);	
}

//...and this runtime function
void motor_stepper_test_runtime(int div)
{
	//Call this function at 1ms intervals. The divider will
	//allow longer delays between steps.
	
	static uint8 hall_code_0 = 0, hall_code = 0;
	static int delay_cnt = 0;
	
	delay_cnt++;
	if(delay_cnt >= div)
	{
		delay_cnt = 0;
	
		hall_code_0++;
		hall_code_0 %= 6;
		hall_code = hall_conv[hall_code_0];
		
		//Hall_Write(hall_code);	//ToDo Enable
	}	
}
