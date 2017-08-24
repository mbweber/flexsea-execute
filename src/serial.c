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
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] serial: code for UART module
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "serial.h"
#include <string.h>
#include "misc.h"
#include "flexsea_comm.h"
#include "user-ex.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t uart_dma_rx_buf[96];	//ToDo #define
uint8_t uart_dma_rx_buf_unwrapped[96];
uint8_t uart_dma_tx_buf[96];
uint8_t uart_dma_bt_rx_buf[96];
uint8_t uart_dma_bt_rx_buf_unwrapped[96];
volatile int8_t tx_cnt = 0;
uint8_t uart_tmp_buf[RX_BUF_LEN];

//DMA:
uint8_t DMA_3_Chan;
uint8_t DMA_3_TD[1];
uint8_t DMA_4_Chan;
uint8_t DMA_4_TD[1];
uint8_t DMA_6_Chan;
uint8_t DMA_6_TD[1];
uint8_t DMA_7_Chan;
uint8_t DMA_7_TD[1];

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void init_dma_3(void);	//RS-485 RX
static void init_dma_4(void);	//RS-485 TX
static void init_dma_6(void);	//Bluetooth RX

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Transmit serial data with DMA
//The DMA transfer is for 48 bytes, by configuration. ToDo: this isn't clean
void rs485_puts(uint8_t *buf, uint32 len)
{
	(void)len; //Unused for now
	
	UART_DMA_XMIT_Write(0);		//No transmission
	UART_2_ClearTxBuffer();		//Clear any old data
	
	CyDelayUs(1);
	NOT_RE_Write(1);			//Disable Receiver
	CyDelayUs(1);
	DE_Write(1);				//Enable Receiver
	CyDelayUs(1);
	//Note: these small delays matter, keep them
	
	//Copy the bytes:
	memcpy(uart_dma_tx_buf, buf, COMM_STR_BUF_LEN);
	
	//Enable channel and UART TX ISR line:
	CyDmaChEnable(DMA_4_Chan, 1);
	UART_DMA_XMIT_Write(1);		//Allow transmission
	
	//DMA will take it from here, go to CY_ISR(isr_dma_uart_tx_Interrupt) for the end
}

//Sends a single character to the UART
void rs485_putc(uint8_t byte)
{
	NOT_RE_Write(1);			//Disable Receiver
	UART_2_PutChar(byte);		//Send byte
}

//Bluetooth puts:
void bt_puts(uint8_t *buf, uint32 len)
{
	UART_1_PutArray(buf, len);
}

//Init UART and transceivers
void init_rs485(void)
{
	#ifdef USE_RS485		

	C8M_SetDividerValue(5);		//16MHz UART clock (2M)
	UART_2_Init();
	UART_2_Enable();
	UART_2_Start();	
	init_dma_3();				//DMA, Reception
	isr_dma_uart_rx_Start();
	init_dma_4();				//DMA, Transmission
	isr_dma_uart_tx_Start();
	NOT_RE_Write(0);			//Enable RS-485 Receiver
	
	//Timer 2: 10us (2 bytes)
	Timer_2_Init();
	Timer_2_Start();
	isr_t2_Start();
	
	#endif	//USE_RS485
}

void init_bluetooth(void)
{
	#ifdef USE_BLUETOOTH
		
	UART_1_Init();
	UART_1_Enable();
	UART_1_Start();

	init_dma_6();				//DMA, Reception
	isr_dma_uart_bt_rx_Start();
	
	#endif //USE_BLUETOOTH
}

//We have a packet ready, but we want to wait a little while before sending it
void rs485DelayedTransmit(PacketWrapper* p)
{
	if(p->destinationPort == PORT_RS485_1)
	{
		//packet[PORT_RS485_1][OUTBOUND] should already be filled
		
		commPeriph[PORT_RS485_1].tx.packetReady = 1;
		//Get reply timestamp from main FSM:
		commPeriph[PORT_RS485_1].tx.timeStamp = (t1_time_share + REPLY_DELAY) % 10;
	}
}

//****************************************************************************
// Test Function(s)
//****************************************************************************

void test_uart_dma_xmit(void)
{
	int i = 0;
	uint8_t databuffer[48];
	
	//Prepare data
	for(i = 0; i < 48; i++)
	{
		databuffer[i] = i;
	}

	while(1)
	{
		UART_DMA_XMIT_Write(0);	//No transmission
		CyDelay(10);	//Wait 10ms
		UART_2_ClearTxBuffer();
		
		//Load data in DMA buffer
		for(i = 0; i < 48; i++)
		{
			uart_dma_tx_buf[i] = databuffer[i];
		}
		
		CyDmaChEnable(DMA_4_Chan, 1);
		UART_DMA_XMIT_Write(1);	//Allow transmission
		CyDelay(1000);	//Wait 10ms
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//DMA3: UART RX (RS-485)
static void init_dma_3(void)
{
	#define DMA_3_BYTES_PER_BURST 		1
	#define DMA_3_REQUEST_PER_BURST 	1
	#define DMA_3_SRC_BASE 				(CYDEV_PERIPH_BASE)
	#define DMA_3_DST_BASE 				(CYDEV_SRAM_BASE)

	DMA_3_Chan = DMA_3_DmaInitialize(DMA_3_BYTES_PER_BURST, DMA_3_REQUEST_PER_BURST, 
	    HI16(DMA_3_SRC_BASE), HI16(DMA_3_DST_BASE));
	DMA_3_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_3_TD[0], 48, DMA_3_TD[0], DMA_3__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(DMA_3_TD[0], LO16((uint32)UART_2_RXDATA_PTR), LO16((uint32)uart_dma_rx_buf));
	CyDmaChSetInitialTd(DMA_3_Chan, DMA_3_TD[0]);
	CyDmaChEnable(DMA_3_Chan, 1);
}

//DMA4: UART TX (RS-485)
static void init_dma_4(void)
{
	#define DMA_4_BYTES_PER_BURST 		1
	#define DMA_4_REQUEST_PER_BURST 	1
	#define DMA_4_SRC_BASE 				(CYDEV_SRAM_BASE)
	#define DMA_4_DST_BASE 				(CYDEV_PERIPH_BASE)
	
	DMA_4_Chan = DMA_4_DmaInitialize(DMA_4_BYTES_PER_BURST, DMA_4_REQUEST_PER_BURST, 
	    HI16(DMA_4_SRC_BASE), HI16(DMA_4_DST_BASE));
	DMA_4_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_4_TD[0], 48, CY_DMA_DISABLE_TD, TD_TERMIN_EN | DMA_4__TD_TERMOUT_EN | TD_INC_SRC_ADR | TD_AUTO_EXEC_NEXT);
	CyDmaTdSetAddress(DMA_4_TD[0], LO16((uint32)uart_dma_tx_buf), LO16((uint32)UART_2_TXDATA_PTR));
	CyDmaChSetInitialTd(DMA_4_Chan, DMA_4_TD[0]);
	CyDmaChEnable(DMA_4_Chan, 1);
}

//DMA6: UART RX (Bluetooth)
static void init_dma_6(void)
{
	#ifdef USE_BLUETOOTH
		
	#define DMA_6_BYTES_PER_BURST 		1
	#define DMA_6_REQUEST_PER_BURST 	1
	#define DMA_6_SRC_BASE 				(CYDEV_PERIPH_BASE)
	#define DMA_6_DST_BASE 				(CYDEV_SRAM_BASE)

	DMA_6_Chan = DMA_6_DmaInitialize(DMA_6_BYTES_PER_BURST, DMA_6_REQUEST_PER_BURST, 
	    HI16(DMA_6_SRC_BASE), HI16(DMA_6_DST_BASE));
	DMA_6_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_6_TD[0], 48, DMA_6_TD[0], DMA_6__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(DMA_6_TD[0], LO16((uint32)UART_1_RXDATA_PTR), LO16((uint32)uart_dma_bt_rx_buf));
	CyDmaChSetInitialTd(DMA_6_Chan, DMA_6_TD[0]);
	CyDmaChEnable(DMA_6_Chan, 1);
	
	#endif	//USE_BLUETOOTH
}
