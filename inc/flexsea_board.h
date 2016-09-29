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
	[This file] flexsea_board: configuration and functions for this 
	particular board
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_FLEXSEA_BOARD_H
#define INC_FLEXSEA_BOARD_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>	
//#include "../../flexsea-comm/inc/flexsea.h"

	
//****************************************************************************
// Prototype(s):
//****************************************************************************

void flexsea_send_serial_slave(unsigned char port, unsigned char *str, unsigned char length);
void flexsea_send_serial_master(unsigned char port, unsigned char *str, unsigned char length);
void flexsea_clear_slave_read_buffer(void);
void build_slave_payload(unsigned char base_addr);
unsigned char flexsea_prepare_rs485_tx_buffer(void);
void rs485_reply_ready(uint8_t *buf, uint32_t len);

//****************************************************************************
// Definition(s):
//****************************************************************************

//<FlexSEA User>
//==============

//Board type - un-comment only one!
//Make sure it matches with board_id!
//#define BOARD_TYPE_FLEXSEA_PLAN
//#define BOARD_TYPE_FLEXSEA_MANAGE
#define BOARD_TYPE_FLEXSEA_EXECUTE

//How many slaves on this bus?
#define SLAVE_BUS_1_CNT				0
#define SLAVE_BUS_2_CNT				0
//Note: only Manage can have a value different than 0 or 1

//Slave Read Buffer Size:
#define SLAVE_READ_BUFFER_LEN		32	//ToDo TBD

//Enabled the required FlexSEA Buffers for this board:
#define ENABLE_FLEXSEA_BUF_1	//RS-485
#define ENABLE_FLEXSEA_BUF_2	//USB
//#define ENABLE_FLEXSEA_BUF_3	//In the future: radio-module
//#define ENABLE_FLEXSEA_BUF_4	//In the future: battery board?

//Overload buffer & function names (for user convenience):

#define comm_str_485_1				comm_str_1
#define unpack_payload_485			unpack_payload_1
#define rx_command_485				rx_command_1
#define update_rx_buf_byte_485		update_rx_buf_byte_1
#define update_rx_buf_array_485		update_rx_buf_array_1

#define comm_str_usb				comm_str_2
#define unpack_payload_usb			unpack_payload_2
#define rx_command_usb				rx_command_2
#define update_rx_buf_byte_usb		update_rx_buf_byte_2
#define update_rx_buf_array_usb		update_rx_buf_array_2

//===============
//</FlexSEA User>

//****************************************************************************
// Structure(s):
//****************************************************************************

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern uint8_t board_id;
extern uint8_t board_up_id;
extern uint8_t board_sub1_id[SLAVE_BUS_1_CNT];
extern uint8_t board_sub2_id[SLAVE_BUS_2_CNT];

#endif	//INC_FLEXSEA_BOARD_H
