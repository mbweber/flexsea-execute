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
	[This file] local_comm: Communication functions, board specific
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-03-06 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "local_comm.h"
#include <flexsea_payload.h>
#include <flexsea_board.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Prepares the structures:
void initLocalComm(void)
{
	//Default state:
	initCommPeriph(&commPeriph[PORT_RS485_1], PORT_RS485_1, MASTER, rx_buf_1, \
			comm_str_1, rx_command_1, &packet[PORT_RS485_1][INBOUND], \
			&packet[PORT_RS485_1][OUTBOUND]);
	initCommPeriph(&commPeriph[PORT_USB], PORT_USB, MASTER, rx_buf_2, \
			comm_str_2, rx_command_2, &packet[PORT_USB][INBOUND], \
			&packet[PORT_USB][OUTBOUND]);
	initCommPeriph(&commPeriph[PORT_WIRELESS], PORT_WIRELESS, MASTER, rx_buf_3, \
			comm_str_3, rx_command_3, &packet[PORT_WIRELESS][INBOUND], \
			&packet[PORT_WIRELESS][OUTBOUND]);

	//Personalize specific fields:
	//...
}

//Did we receive new commands? Can we parse them?
void parseMasterCommands(uint8_t *new_cmd)
{
	uint8_t parseResult = 0, newCmdLed = 0;
	
	//RS-485
	if(commPeriph[PORT_RS485_1].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_RS485_1].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_RS485_1][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
		CyDmaClearPendingDrq(DMA_3_Chan);
	}

	//USB
	if(commPeriph[PORT_USB].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_USB].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_USB][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
	}

	//Wireless
	if(commPeriph[PORT_WIRELESS].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_WIRELESS].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_WIRELESS][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
	}

	if(newCmdLed > 0) {*new_cmd = 1;}
}

//Call this to send any pending delayed reply on RS-485
void sendMasterDelayedResponse(void)
{
	Port port = PORT_RS485_1;
	
	if(commPeriph[port].tx.packetReady == 1)
	{
		//We never replied in the same time slot:
		if(t1_time_share == commPeriph[port].tx.timeStamp)
		{
			//rs485_puts(reply_ready_buf, reply_ready_len);
			rs485_puts(packet[port][OUTBOUND].packed, \
						packet[port][OUTBOUND].numb);
			
			//Drop flag
			commPeriph[port].tx.packetReady = 0;
		}		
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
