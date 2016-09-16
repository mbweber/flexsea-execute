//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@media.mit.edu
// 02/2016
//****************************************************************************
// isr_callbacks: Implementation of the ISR functions
//****************************************************************************

//Important: this files goes with "cyapicallbacks.h". PSoC Creator 3.3 doesn't 
//include "cyapicallbacks.h" if I place it in /inc/... or I'm doing something
//wrong. Solution was to place it in /execute_1_0.cydsn/.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "cyapicallbacks.h"

//****************************************************************************
// Public Function(s)
//****************************************************************************

void isr_t1_Interrupt_InterruptCallback()
{
	//Timer 1: 100us
	
	//Clear interrupt
	Timer_1_ReadStatusRegister();
	isr_t1_ClearPending();
	
	//All the timings are based on 100us slots
	//10 slots form the original 1ms timebase
	//'t1_time_share' is from 0 to 9, it controls the main FSM
	
	t1_new_value = 1;
	
	//Flag for the main code
	t1_100us_flag = 1;	
}

void isr_t2_Interrupt_InterruptCallback()
{
	//Timer 2: 10us delay. RS-485 transceiver.
	
	//Clear interrupt
	Timer_2_ReadStatusRegister();
	isr_t2_ClearPending();	
	
	//Transfer is over, enable Receiver & disable Emitter
	UART_DMA_XMIT_Write(0);		//No transmission
	DE_Write(0);
	//CyDelayUs(1);
	NOT_RE_Write(0);
	
	T2_RESET_Write(1);	
}

//General ADC - we get here after 8 samples have been taken
void isr_sar1_dma_Interrupt_InterruptCallback()
{
	static unsigned char ch = 0;
	int i = 0;
	
	//Stop conversion
	ADC_SAR_1_StopConvert();

	//Copy the last DMA buffer to our 2D array:
	for(i = 0; i < ADC1_BUF_LEN; i++)
	{
		adc1_res[ch][i] = adc_sar1_dma_array[i+1];
	}	

	//Next channel:
	ch++;
	ch %= ADC1_CHANNELS;
	
	//Once we have all the channels we copy the buffer:
	if(!adc_sar1_flag)
	{
		double_buffer_adc();
	}
	adc_sar1_flag = 1;

	//Refresh MUX:
	AMux_1_Select(ch);	
	
	ADC_SAR_1_StartConvert();		
}

//Current sensing:
void isr_sar2_dma_Interrupt_InterruptCallback()
{
	volatile int32 adc_sum = 0;
	volatile int32 adc_avg = 0;
	
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
		
	//Read last ADC value
	adc_sum = (int32)(adc_dma_array[0] + adc_dma_array[1] + adc_dma_array[2] + \
				adc_dma_array[3] + adc_dma_array[4]);
	adc_avg = (adc_sum / 5);
	
	ctrl.current.actual_val = (int32)(adc_avg - CURRENT_ZERO);	
	//Used by the current controller, 0 centered.
		
	if((ctrl.active_ctrl == CTRL_CURRENT) || (ctrl.active_ctrl == CTRL_IMPEDANCE))
	{
		//Current controller
		motor_current_pid_2(ctrl.current.setpoint_val, ctrl.current.actual_val);
	}
	
	#else
		
    update_current_arrays();
		
	#endif
}


void isr_dma_uart_rx_Interrupt_InterruptCallback()
{
	//Update rx_buf with the latest DMA data:
	unwrap_buffer(uart_dma_rx_buf, uart_dma_rx_buf_unwrapped, 48);
	update_rx_buf_array_485(uart_dma_rx_buf_unwrapped, 48);		//ToDo shouldn't be harcoded. Buffer name?
	data_ready_485++;
}

void isr_dma_uart_tx_Interrupt_InterruptCallback()
{
	T2_RESET_Write(0);
	Timer_2_Start();		
}

void isr_delsig_Interrupt_InterruptCallback()
{
	ADC_DelSig_1_StopConvert();
	adc_delsig_flag = 1;	
}

void ADC_SAR_1_ISR_InterruptCallback()
{
	//Not used anymore
}

//SPI - AS5047 position sensor
void isr_spi_tx_Interrupt_InterruptCallback()
{
	#ifdef USE_AS5047
    
	//static volatile uint16 frame_errors = 0, parity_errors = 0, man_test = 0;
	volatile uint8 tx_status_isr = 0;
	
	//Read status to clear flag:
	tx_status_isr = SPIM_1_ReadTxStatus();
	
	if(spi_isr_state < SPI_TX_MAX_INDEX)
	{
		//Read last word received:
		spidata_miso[spi_isr_state] = SPIM_1_ReadRxData();
		//Next transfer:
		spi_isr_state++;
		SPIM_1_WriteTxData(as5047_empty_read);
	}
	else
	{
		//Transfer complete, decode answer:
        spi_read_flag = 1;

		
        /* Partially developped error testing code:
		
		//Error in last command frame?
		if(spidata_miso[spi_isr_state] & AS5047_ERR_FRAME)
		{
			frame_errors++;
		}
		
		if(as5047_angle == 0)
		{
			man_test++;
			//EX15_Write(1);
			//EX15_Write(0);
		}
		//Wrong parity?		
		//...
		*/
        
	}   
	#endif	//USE_AS5047
}
