//****************************************************************************
// Dephy, Inc.
// Jean-Francois (JF) Duval
// jfduval@dephy.compare
// 08/2016
//****************************************************************************
// mem_angle: Non-volatile Angle table R/W
//****************************************************************************
	
#ifndef INC_MEM_ANG_H
#define INC_MEM_ANG_H

/*
Notes - EEPROM:
===============
EEPROM can be written one row (16 bytes) at the time. Our angle table is
126*uint16 = 252 bytes. 252/16 = 15.75 = 16 rows.
It can take up to 20ms per write, so 320ms total.
We start at address 0.
	
Notes - FLASH:
==============
Code appears to work... but the Program operation starts by erasing all the
FLASH so the value of this code is very limited...
*/

//****************************************************************************
// Include(s)
//****************************************************************************		
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************	

void init_eeprom(void);
void save_angles_to_eeprom(uint16 *new_angles);
void load_angles_from_eeprom(uint16 *ee_angles);
void test_angle_eeprom(void);

void init_flash(void);
void save_angles_to_flash(uint16 *new_angles, uint16 datapoints, uint8 arr);
void load_angles_from_flash(uint16 *ee_angles, uint16 datapoints, uint8 arr);
void test_angle_flash(void);

//****************************************************************************
// Definition(s):
//****************************************************************************	

//EEPROM:
#define EE_ROW_LEN_WORD			8
#define EE_ROW_LEN_BYTES		(EE_ROW_LEN_WORD*2)
#define EE_ANGLE_MAX_ROW		16
#define EE_SIZE_BYTES			(EE_ROW_LEN_BYTES * EE_ANGLE_MAX_ROW)

//FLASH:
#define FLASH_MAX_DATAPOINTS	250	//2048

//Test code:
#define TEST_DATA_LEN			128		//Do not change
#define DATA_INC				300

//****************************************************************************
// Structure(s)
//****************************************************************************	

	
#endif	//INC_MEM_ANG_H
	