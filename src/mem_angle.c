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
	[This file] mem_angle: Non-volatile Angle table R/W
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "mem_angle.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

const uint8_t flash_angle_array_0[FLASH_MAX_DATAPOINTS] __attribute__((section(".angletables")));// = {0,0,0};
//const uint8_t flash_angle_array_0[FLASH_MAX_DATAPOINTS] = {0,0,0};
const uint8_t flash_angle_array_1[FLASH_MAX_DATAPOINTS] __attribute__((section(".angletables")));// = {0,0,0};
const uint8_t flash_angle_array_2[FLASH_MAX_DATAPOINTS] __attribute__((section(".angletables")));// = {0,0,0};

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	


//****************************************************************************
// Public Function(s)
//****************************************************************************

//=======
//EEPROM:
//=======

//EEPROM used to save motor angles
void init_eeprom(void)
{
	EEPROM_1_Start();
	CyDelayUs(5);	//Needs 5us to start
}	

//Saves an angle table to EEPROM - use that when calibrating
void save_angles_to_eeprom(uint16_t *new_angles)
{
	uint8_t row_cnt = 0, word_cnt = 0;
	uint8_t ang_in_bytes[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	cystatus status = CYRET_UNKNOWN;
	uint16_t new_word = 0;
	
	//Update temperature reading:
	EEPROM_1_UpdateTemperature();
	
	//One row at the time:
	for(row_cnt = 0; row_cnt < EE_ANGLE_MAX_ROW; row_cnt++)
	{
		//From 8x uint16_t to 16x uint8_t:
		for(word_cnt = 0; word_cnt < 8; word_cnt++)
		{
			new_word = new_angles[(EE_ROW_LEN_WORD * row_cnt) + word_cnt];			
			ang_in_bytes[word_cnt << 1] = (new_word & 0xFF00) >> 8;	//MSB
			ang_in_bytes[(word_cnt << 1) + 1] = (new_word & 0xFF);	//LSB
		}
		
		//Write row:
		status = EEPROM_1_Write(ang_in_bytes, row_cnt);
		//(blocking, ~20ms)
	}
}

//Reads an angle table from EEPROM - use that for normal operation
void load_angles_from_eeprom(uint16_t *ee_angles)
{
	uint16_t cnt = 0;
	uint16_t msb = 0, lsb = 0;
	
	for(cnt = 0; cnt < (EE_SIZE_BYTES/2); cnt++)
	{
		//Get word from 2 bytes:
		msb = (EEPROM_1_ReadByte(cnt << 1) << 8) & 0xFF00;
		lsb = EEPROM_1_ReadByte((cnt << 1) + 1) & 0x00FF;
		ee_angles[cnt] = msb | lsb;
	}
}

//Writes human-readable data, and read it back
void test_angle_eeprom(void)
{
	uint16_t test_w_data[TEST_DATA_LEN], test_r_data[TEST_DATA_LEN];
	uint16_t cnt = 0;
	uint16_t val = 0;
	volatile uint16_t error_cnt = 0;
	
	//Fill tables:
	for(cnt = 0; cnt < TEST_DATA_LEN; cnt++)
	{
		//'Write' table:
		test_w_data[cnt] = val;
		val = val + DATA_INC;
		
		//'Read' table - all zeros
		test_r_data[cnt] = 0;
	}
	
	//Save it:
	save_angles_to_eeprom(test_w_data);
	
	//And read it back:
	load_angles_from_eeprom(test_r_data);
	
	//Compare:
	for(cnt = 0; cnt < TEST_DATA_LEN; cnt++)
	{
		if(test_w_data[cnt] != test_r_data[cnt])
		{
			error_cnt++;
		}		
	}
}

//======
//FLASH:
//======

void init_flash(void)
{
	Em_EEPROM_1_Start();
}

//Saves an angle table to FLASH - use that when calibrating
//'datapoints' indicated the number of words. It has to be smaller than 
//'FLASH_MAX_DATAPOINTS'. arr is 0, 1 or 2.
void save_angles_to_flash(uint16_t *new_angles, uint16_t datapoints, uint8_t arr)
{
	cystatus status = CYRET_UNKNOWN;
	uint8_t tmp_buf_bytes[2*FLASH_MAX_DATAPOINTS];
	const uint8_t *flashPtr;
	int i = 0;
	
	//Assign pointer to array:
	switch(arr)
	{
		case 0:	
			flashPtr = flash_angle_array_0;
			break;
		case 1:	
			flashPtr = flash_angle_array_1;
			break;
		case 2:	
			flashPtr = flash_angle_array_2;
			break;
		default:	
			flashPtr = flash_angle_array_0;
			break;
	}
	
	//Chops words into bytes:
	for(i = 0; i < datapoints; i++)
	{
		tmp_buf_bytes[i << 1] = (new_angles[i] & 0xFF00) >> 8;	//MSB
		tmp_buf_bytes[(i << 1) + 1] = (new_angles[i] & 0xFF);	//LSB
	}
	
	//Write to FLASH:
	status = Em_EEPROM_1_Write(tmp_buf_bytes, flashPtr, 2*datapoints);
	
	/*
	//Traps - debugging only
	if(status == CYRET_BAD_PARAM)
		while(1);
	if(status == CYRET_UNKNOWN)
		while(1);
	*/
}

//Reads an angle table from FLASH - use that for normal operation
void load_angles_from_flash(uint16_t *ee_angles, uint16_t datapoints, uint8_t arr)
{
	uint16_t cnt = 0;
	uint16_t msb = 0, lsb = 0;
	volatile const uint8_t *flashPtr;
	
	//Assign pointer to array:
	switch(arr)
	{
		case 0:	
			flashPtr = flash_angle_array_0;
			break;
		case 1:	
			flashPtr = flash_angle_array_1;
			break;
		case 2:	
			flashPtr = flash_angle_array_2;
			break;
		default:	
			flashPtr = flash_angle_array_0;
			break;
	}
	
	for(cnt = 0; cnt < datapoints; cnt++)
	{
		//Get word from 2 bytes:
		msb = (*(flashPtr + (cnt << 1)) << 8) & 0xFF00;
		lsb = (*(flashPtr + ((cnt << 1) + 1))) & 0x00FF;
		ee_angles[cnt] = msb | lsb;
	}
}

//Writes human-readable data, and read it back
void test_angle_flash(void)
{
	uint16_t test_w_data[TEST_DATA_LEN], test_r_data[TEST_DATA_LEN];
	uint16_t cnt = 0;
	uint16_t val = 0;
	volatile uint16_t error_cnt = 0;
	
	//Fill tables:
	for(cnt = 0; cnt < TEST_DATA_LEN; cnt++)
	{
		//'Write' table:
		test_w_data[cnt] = val;
		val = val + DATA_INC;
		
		//'Read' table - all zeros
		test_r_data[cnt] = 0;
	}
	
	//Save it:
	//save_angles_to_flash(test_w_data, TEST_DATA_LEN, 0);
	//save_angles_to_flash(test_w_data, TEST_DATA_LEN, 1);
	//save_angles_to_flash(test_w_data, TEST_DATA_LEN, 2);
	
	//And read it back:
	load_angles_from_flash(test_r_data, TEST_DATA_LEN, 0);
	
	//Compare:
	for(cnt = 0; cnt < TEST_DATA_LEN; cnt++)
	{
		if(test_w_data[cnt] != test_r_data[cnt])
		{
			error_cnt++;
		}		
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
