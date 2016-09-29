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

#ifndef INC_SERIAL_H
#define INC_SERIAL_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_rs485(void);
void init_bluetooth(void);
void rs485_putc(uint8 byte);
void rs485_puts(uint8 *buf, uint32 len);
void rs485_isr_puts(uint8 *buf, uint32 len);
void rs485_dma_puts(uint8 *buf);
void test_uart_dma_xmit(void);
void t2_oneshot_test(void);
void get_uart_data(void);

//****************************************************************************
// Shared Variable(s):
//****************************************************************************

extern uint8 uart_dma_rx_buf[96];
extern uint8 uart_dma_tx_buf[96];

extern uint8 reply_ready_buf[96];
extern uint8 reply_ready_flag;
extern uint8 reply_ready_len;
extern uint8 reply_ready_timestamp;

extern uint8 DMA_3_Chan;

//****************************************************************************
// Definition(s):
//****************************************************************************
	
#endif	//INC_SERIAL_H
	