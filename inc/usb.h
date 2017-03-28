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
	[This file] usb: USB CDC
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/
	
#ifndef INC_USB_H
#define INC_USB_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

uint8_t init_usb(void);
void send_usb_int8_t(char payload);
void send_usb_uint8_t(uint8_t payload);
void send_usb_int16(int16 payload);
void send_usb_int32(int payload);
void get_usb_data(void);
void usb_puts(uint8_t *buf, uint32 len);
void usbRuntimeConnect(void);

uint8_t usb_echo_blocking(void);

int16 send_usb_packet(uint16 word1, uint16 word2, uint16 word3, uint16 word4);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define USB_ENUM_TIMEOUT			2500	//ms
#define BUFFER_LEN					64
	
#endif	//INC_USB_H
