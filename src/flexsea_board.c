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

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "flexsea_board.h"
#include "../../flexsea-system/inc/flexsea_system.h"
#include "user-ex.h"
#include "local_comm.h"
#include "serial.h"
#include "usb.h"
#include <flexsea_comm.h>
#include <flexsea_payload.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

//Board ID (this board) - pick from Board list in flexsea_system.h
uint8_t board_id = FLEXSEA_EXECUTE_1;	//Can be modified in in user.c/h!
uint8_t board_up_id = FLEXSEA_MANAGE_1;
uint8_t board_sub1_id[SLAVE_BUS_1_CNT];
uint8_t board_sub2_id[SLAVE_BUS_2_CNT];

//****************************************************************************
// Function(s)
//****************************************************************************

//Wrappers for the specific serial functions. Useful to keep flexsea_network
//platform independent (for example, we don't need need puts_rs485() for Plan)

//Communication with a slave
void flexsea_send_serial_slave(PacketWrapper* p)
{
	//Execute doesn't have slaves:
	(void)p;
}

//Communication with our master
void flexsea_send_serial_master(PacketWrapper* p)
{
	Port port = p->destinationPort;
	uint8_t *str = p->packed;
	uint16_t length = COMM_STR_BUF_LEN;
	
	if(port == PORT_RS485_1)
	{
		//Delayed response:
		#ifdef USE_RS485
			
			rs485DelayedTransmit(p);
		
		#endif 	//USE_RS485
	}
	else if(port == PORT_USB)
	{
		#ifdef USE_USB
			
			usb_puts(str, length);
			
		#endif
	}
	else if(port == PORT_WIRELESS)
	{
		//Delayed response:
		#ifdef USE_BLUETOOTH
			
			bt_puts(str, length);
			
		#endif 	//USE_BLUETOOTH
	}
	else
	{
		//Deal with errors here ToDo
	}
}

void flexsea_receive_from_master(void)
{
	#ifdef USE_USB			

		get_usb_data();
		tryUnpacking(&commPeriph[PORT_USB], &packet[PORT_USB][INBOUND]);
		
	#endif

	#ifdef USE_RS485
		
		tryUnpacking(&commPeriph[PORT_RS485_1], &packet[PORT_RS485_1][INBOUND]);
		
	#endif

	#ifdef USE_BLUETOOTH
		
		tryUnpacking(&commPeriph[PORT_WIRELESS], &packet[PORT_WIRELESS][INBOUND]);
		
	#endif
}

uint8_t getBoardID(void)
{
	return board_id;
}

void setBoardID(uint8_t bid)
{
	board_id = bid;
}

uint8_t getBoardUpID(void)
{
	return board_up_id;
}

uint8_t getBoardSubID(uint8_t sub, uint8_t idx)
{
	if(sub == 0){return board_sub1_id[idx];}
	else if(sub == 1){return board_sub2_id[idx];}

	return 0;
}

uint8_t getSlaveCnt(uint8_t sub)
{
	if(sub == 0){return SLAVE_BUS_1_CNT;}
	else if(sub == 1){return SLAVE_BUS_2_CNT;}

	return 0;
}
