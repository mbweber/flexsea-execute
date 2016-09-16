//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@mit.edu
// 02/2015
//****************************************************************************
// peripherals: code for the general peripheral modules
//****************************************************************************

//Note:  many init_x() functions are included in other files to keep a logical
// organization. Place them here if they are not specific to one file.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "peripherals.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8 uart_dma_rx_buf[96];	//ToDo #define
uint8 uart_dma_rx_buf_unwrapped[96];
uint8 uart_dma_tx_buf[96];
uint8 DMA_4_Chan;
uint8 DMA_4_TD[1];
uint8 gui_fsm_flag = DISABLED;

int32 last_ang_read_period;
int32 counts_since_last_ang_read;
int32 angtimer_read = 65000, last_angtimer_read = 65000;

//****************************************************************************
// Function(s)
//****************************************************************************

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
	
	//Start with an empty buffer
	flexsea_clear_slave_read_buffer();
	
	#if(MOTOR_COMMUT == COMMUT_SINE) 
	//Start converting:
	ADC_SAR_2_StartConvert();
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

//update the number of counts since the last time the angle sensor was read
void update_counts_since_last_ang_read(void)
{
    angtimer_read = Timer_angleread_ReadCounter(); 
    
    if (angtimer_read>last_angtimer_read+20000)
    {
        counts_since_last_ang_read =  counts_since_last_ang_read + last_angtimer_read+65000-angtimer_read;
    }
    else
    {
        counts_since_last_ang_read = counts_since_last_ang_read + last_angtimer_read-angtimer_read;
    }
    last_angtimer_read = angtimer_read;
    
}

//resent the angle read counter and store the the counts between last two readings
void reset_ang_counter(void)
{
    update_counts_since_last_ang_read();
    last_ang_read_period = counts_since_last_ang_read-0;
    counts_since_last_ang_read = 0;
}

//update all of the angle variables
void update_as5047(int32 ang)
{
    //shift angle arrays
    int ii = 9;
    for (; ii>0;ii--)
    {
        as5047.angle_raws[ii] = as5047.angle_raws[ii-1];
        as5047.angle_conts[ii] = as5047.angle_conts[ii-1];
    }
    
    //assign latest raw angle value
    as5047.angle_raws[0] = ang;
    
    //determine if the encoder has rotated past 0/18383 point and in what direction
    if ((as5047.angle_raws[0]-as5047.angle_raws[1])<-5000)
    {
        as5047.num_rot++;
        EX1_Write(!EX1_Read());
    }
    else if ((as5047.angle_raws[0]-as5047.angle_raws[1])>5000)
    {
        as5047.num_rot--;
        EX1_Write(!EX1_Read());
    }
    
    //calculate the continuous value of the encoder 
    as5047.angle_conts[0] = 16384*as5047.num_rot+as5047.angle_raws[0]; 
    
    //calculate the simple difference velocity
    as5047.angle_vel[1] = as5047.angle_vel[0];
    as5047.angle_vel[0] = as5047.angle_conts[0]-as5047.angle_conts[8];
    as5047.angle_vel_filt[1] = as5047.angle_vel_filt[0];
    filt_array(as5047.angle_vel, as5047.angle_vel_filt);
    
    //sum the 1 MHz clicks between 8 readings of the angle sensor
    int angle_vel_denom_sum = 0;
    for (ii=7;ii>0;ii--)
    {
        as5047.angle_vel_denoms[ii] = as5047.angle_vel_denoms[ii-1];
        angle_vel_denom_sum += as5047.angle_vel_denoms[ii];
    }
    as5047.angle_vel_denoms[0] = last_ang_read_period;
    angle_vel_denom_sum += as5047.angle_vel_denoms[0];

    //caluclate the filtered RPM of the motor
    as5047.angle_vel_RPMS_raw[1] = as5047.angle_vel_RPMS_raw[0];
    as5047.angle_vel_RPMS_raw[0] = ((as5047.angle_vel_filt[0]*3662)/angle_vel_denom_sum)>>10; //clicks in 8 samples / 1 Mhz clicks in 8 samples * 3662. 3662 =  1000000 Hz /16384 clicks/rot *60 sec/min  
    as5047.angle_vel_RPMS_filt[1] = as5047.angle_vel_RPMS_filt[0];
    filt_array( as5047.angle_vel_RPMS_raw, as5047.angle_vel_RPMS_filt);
    as5047.angle_vel_RPM = as5047.angle_vel_RPMS_filt[0]>>10;
    
    //update the 1 MHz counts from the last angle read
    update_counts_since_last_ang_read();
    
    //Update the velocity compensated angle
    as5047.angle_comp = ((((counts_since_last_ang_read+90)*(as5047.angle_vel_filt[0]>>10))/angle_vel_denom_sum+(as5047.angle_raws[0])+16384)%16384);
    
    global_variable_1 = as5047.angle_vel_RPM;
}
