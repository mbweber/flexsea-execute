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
	[This file] main_fsm: Contains all the case() code for the main FSM
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "main_fsm.h"
#include "ext_input.h"
#include "flexsea_global_structs.h"
#include "calibration_tools.h"
//****************************************************************************
// Variable(s)
//****************************************************************************

uint8 eL0 = 0, eL1 = 0, eL2 = 0;
uint16 safety_delay = 0;
uint8 new_cmd_led = 0;
uint8_t tmp_rx_command_485[PAYLOAD_BUF_LEN];
uint8_t tmp_rx_command_usb[PAYLOAD_BUF_LEN];
uint8_t tmp_rx_command_wireless[PAYLOAD_BUF_LEN];
uint8 toggle_wdclk = 0;	
uint8 cmd_ready_485 = 0, cmd_ready_usb = 0, cmd_ready_wireless = 0;	
int steps = 0, current_step = 0;
int spi_read_flag = 0;

//***Test Code - ToDo Remove ***
uint8_t myStr[48] = "abcdefghijklmnopqrstuvwxyz1234567890ABCD";

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	

//****************************************************************************
// Public Function(s)
//****************************************************************************

//1kHz time slots:
//================

//Case 0: I2C_0
void main_fsm_case_0(void)
{
	i2c_0_fsm();
}

//Case 1: I2C_1
void main_fsm_case_1(void)
{
	//Read from Safety Co-Processor
	#ifdef USE_I2C_1
		
	safety_cop_read_all();
	
	#endif 	//USE_I2C_1
}

//Case 2: some safety features & 100ms timebase
void main_fsm_case_2(void)
{
	#ifdef USE_I2T_LIMIT
	//Sample current (I2t limit):
	i2t_sample(ctrl.current.actual_val);
	#endif	//USE_I2T_LIMIT
			
	//100ms timebase:
	if(timebase_100ms())
	{
		#ifdef USE_I2T_LIMIT
		//Is the current in range?
		currentLimit = i2t_compute();
		#endif	//USE_I2T_LIMIT
	} 
}

//Case 3: Strain Gauge DelSig ADC, SAR ADC
void main_fsm_case_3(void)
{
	#ifdef USE_STRAIN
	//Start a new conversion
	ADC_DelSig_1_StartConvert();
	
	//Filter the previous results
	strain_filter_dma();
	#endif
}

//Case 4: User Interface
void main_fsm_case_4(void)
{
	//Alive LED
	alive_led();
	
	//UI RGB LED:
	
	rgbLedRefreshFade();
	
	if(safety_delay > SAFETY_DELAY)
	{
		//status_error_codes(safety_cop.status1, safety_cop.status2, &eL0, &eL1, &eL2); 
	}
	else
	{
		safety_delay++;
	}
	
	//Display temperature status on RGB	
	overtemp_error(&eL1, &eL2);					//Comment this line if safety code is problematic
	rgb_led_ui(eL0, eL1, eL2, new_cmd_led);		//ToDo add more error codes
	if(new_cmd_led)
	{
		new_cmd_led = 0;
	}
}

//Case 5: Position sensors & Position setpoint
void main_fsm_case_5(void)
{
	//Refresh encoder readings (ENC_CONTROL only)
	refresh_enc_control();
	
	#ifdef USE_TRAPEZ	

	//Trapezoidal trajectories (can be used for both Position & Impedance)	
	if((ctrl.active_ctrl == CTRL_POSITION) || (ctrl.active_ctrl == CTRL_IMPEDANCE))
	{									
		ctrl.position.trap_t++;
        ctrl.impedance.trap_t++;
        ctrl.position.setp = trapez_get_pos(steps);	//New setpoint
		ctrl.impedance.setpoint_val = trapez_get_pos(steps);	//New setpoint
	}
	
	#endif	//USE_TRAPEZ    
}

//Case 6: P & Z controllers, 0 PWM
void main_fsm_case_6(void)
{
	//#ifdef USE_TRAPEZ	
	
	// If we are running a calibration test, all controllers should be disabled anyways.
	// Also we should be in CTRL_NONE, but that should be handled elsewhere
	if(calibrationFlags != 0)
	{
		return;
	}
	
	if(ctrl.active_ctrl == CTRL_POSITION)
	{
		motor_position_pid(ctrl.position.setp, ctrl.position.pos);
	}
	else if(ctrl.active_ctrl == CTRL_IMPEDANCE)
	{
        impedance_controller();
	}
	
	//#endif	//USE_TRAPEZ
	
	//If no controller is used the PWM should be 0:
	if(ctrl.active_ctrl == CTRL_NONE)
	{
		motor_open_speed_1(0);
	}
}

//Case 7:
void main_fsm_case_7(void)
{}

//Case 8: SAR ADC filtering
void main_fsm_case_8(void)
{
	if(adc_sar1_flag)
	{
		filter_sar_adc();
		adc_sar1_flag = 0;
	}	
}

//Case 9: User functions & 1s timebase	
void main_fsm_case_9(void)
{    
	if(calibrationFlags & CALIBRATION_FIND_POLES)
	{
		find_poles();
		if(!findingpoles)
		{
			calibrationFlags = 0;
		}
	}
	else
	{
		#if(RUNTIME_FSM == ENABLED)
			user__fsm();
		#endif
	}
    
	//1s timebase:
	if(timebase_1s())
	{
		//Insert code that needs to run every second here
		//...
		
		//***Test Code - ToDo Remove ***
		bt_dma_puts(myStr);
	} 
}

//10kHz time slot:
//================

void main_fsm_10kHz(void)
{    
	uint8 i = 0;
	uint8_t result = 0;
	uint8_t info[2] = {0,0};
	
	#if(MOTOR_COMMUT == COMMUT_SINE) 
	    //send command to read the as5047 angle
	    #if(ENC_COMMUT == ENC_AS5047)
			as5047_read_single_isr(AS5047_REG_ANGLECOM);	//Start reading, result via ISR  
		#endif //ENC_AS5047
	       
		//set encoder reader timer to 0  
	#endif	//(MOTOR_COMMUT == COMMUT_SINE) 
	
	//RS-485 Byte Input
	#ifdef USE_RS485	

	//Data received via DMA
	if(data_ready_485)
	{
		data_ready_485 = 0;
		//Got new data in, try to decode
		cmd_ready_485 = unpack_payload_485();
	}
		
	#endif	//USE_RS485
	
	//Bluetooth Byte Input
	#ifdef USE_BLUETOOTH	

	//Data received via DMA
	if(data_ready_wireless)
	{
		data_ready_wireless = 0;
		//Got new data in, try to decode
		cmd_ready_wireless = unpack_payload_wireless();
	}
		
	#endif	//USE_BLUETOOTH
	
	//USB Byte Input
	#ifdef USE_USB			

	get_usb_data();
	
	if(data_ready_usb)
	{
		data_ready_usb = 0;
		//Got new data in, try to decode
		cmd_ready_usb = unpack_payload_usb();
		
		eL1 = 1;
	}

	#endif	//USE_USB
	
	//FlexSEA Network Communication
	#ifdef USE_COMM
		
	//Valid communication from RS-485?
	if(cmd_ready_485 != 0)
	{
		cmd_ready_485 = 0;
		
		//Cheap trick to get first line	//ToDo: support more than 1
		//ToDo: use memcpy
		for(i = 0; i < PAYLOAD_BUF_LEN; i++)
		{
			tmp_rx_command_485[i] = rx_command_485[0][i];
		}
		
		//payload_parse_str() calls the functions (if valid)<
		info[0] = PORT_485_1;
		result = payload_parse_str(tmp_rx_command_485, info);
		
		//LED:
		if(result == PARSE_SUCCESSFUL)
		{
			//Green LED only if the ID matched and the command was known
			new_cmd_led = 1;
		}
		
		//Test ToDo remove
		CyDmaClearPendingDrq(DMA_3_Chan);
	}
	
	//Time to reply - RS-485?
	if(reply_ready_flag)
	{
		//We never replied in the same time slot:
		if(t1_time_share != reply_ready_timestamp)
		{
			rs485_puts(reply_ready_buf, reply_ready_len);	
		
			reply_ready_flag = 0;
		}		
	}

	//Valid communication from USB?
	if(cmd_ready_usb != 0)
	{
		cmd_ready_usb = 0;
		
		//Cheap trick to get first line	//ToDo: support more than 1
		for(i = 0; i < PAYLOAD_BUF_LEN; i++)
		{
			tmp_rx_command_usb[i] = rx_command_usb[0][i];
		}
		
		//payload_parse_str() calls the functions (if valid)
		info[0] = PORT_USB;
		result = payload_parse_str(tmp_rx_command_usb, info);
		
		//LED:
		if(result == PARSE_SUCCESSFUL)
		{
			//Green LED only if the ID matched and the command was known
			new_cmd_led = 1;
		}
	}
	
	#ifdef USE_BLUETOOTH
	
		//Valid communication from Bluetooth?
		if(cmd_ready_wireless != 0)
		{
			cmd_ready_wireless = 0;
			
			//Cheap trick to get first line	//ToDo: support more than 1
			//ToDo: use memcpy
			for(i = 0; i < PAYLOAD_BUF_LEN; i++)
			{
				tmp_rx_command_wireless[i] = rx_command_wireless[0][i];
			}
			
			//payload_parse_str() calls the functions (if valid)
			info[0] = PORT_WIRELESS;
			result = payload_parse_str(tmp_rx_command_wireless, info);
			
			//LED:
			if(result == PARSE_SUCCESSFUL)
			{
				//Green LED only if the ID matched and the command was known
				new_cmd_led = 1;
			}
			
			//Test ToDo remove
			//CyDmaClearPendingDrq(DMA_3_Chan);
		}
	
	#endif
	
	#endif	//USE_COMM 
	
	#if(((MOTOR_COMMUT == COMMUT_BLOCK) && (CURRENT_SENSING != CS_LEGACY)) || \
		(MOTOR_COMMUT == COMMUT_SINE))
		
		current_rms_1();	//update the motor current
		
    	if((calibrationFlags == 0) && ((ctrl.active_ctrl == CTRL_CURRENT) || (ctrl.active_ctrl == CTRL_IMPEDANCE)))
    	{
    		//Current controller
    		motor_current_pid_3(ctrl.current.setpoint_val, ctrl.current.actual_val);
    	}
		
	#endif
	
	#if(MOTOR_COMMUT == COMMUT_SINE)                    

        //wait until the 2nd ISR callback lifts spi_read_flag 
        int tt = 0;
        while (!spi_read_flag || tt>10)
        {
            tt++;
            CyDelayUs(1);
        }
        reset_ang_counter(&as5047); //reset the counter from the last time an angle was read
        
        //read as5047 encoder data from memory
        spidata_miso[spi_isr_state] = SPIM_1_ReadRxData();
        as5047_angle = (spidata_miso[spi_isr_state] & 0x3FFF);
        spi_read_flag = 0;
        update_as504x(as5047_angle, &as5047);
        
        sensor_sin_commut(as5047.ang_comp_clks>>3, exec1.sine_commut_pwm);
	#endif	//(MOTOR_COMMUT == COMMUT_SINE)
	
	//RGB LED:
	rgbLedRefresh();
}

//Asynchronous time slots:
//========================

void main_fsm_asynchronous(void)
{
	//WatchDog Clock (Safety-CoP)
	toggle_wdclk ^= 1;
	WDCLK_Write(toggle_wdclk);
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
