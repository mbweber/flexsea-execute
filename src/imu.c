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
	[Contributors] Erin Main (ermain@mit.edu)
*****************************************************************************
	[This file] fm_i2c: IMU configuration
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-23 | jfduval | Initial GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "imu.h"
#include "i2c.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

volatile uint8_t i2c_tmp_buf[IMU_MAX_BUF_SIZE];
struct imu_s imu;

//****************************************************************************
// Private Function Prototype(s)
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

// Initialize the IMU w/ default values in config registers
void init_imu(void) 
{
	//Reset the IMU
	reset_imu();
	CyDelay(25);
	
	//Initialize the config registers.
	uint8_t config[4] = { D_IMU_CONFIG, D_IMU_GYRO_CONFIG, D_IMU_ACCEL_CONFIG, \
							D_IMU_ACCEL_CONFIG2 };
	
	//Send the config sequence
	imu_write(IMU_CONFIG, config, 4);
}

// Reset the IMU to default settings
void reset_imu(void) 
{
	uint8_t config = D_DEVICE_RESET;
	imu_write(IMU_PWR_MGMT_1, &config, 1);
}

// Get accel X
int16_t get_accel_x(void) 
{
	uint8_t data[2] = { 0, 0 };
	i2c0_read(IMU_ADDR, IMU_ACCEL_XOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}
// Get accel Y
int16_t get_accel_y(void)
{
	uint8_t data[2];
	i2c0_read(IMU_ADDR, IMU_ACCEL_YOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}

// Get accel Z
int16_t get_accel_z(void)
{
	uint8_t data[2];
	i2c0_read(IMU_ADDR, IMU_ACCEL_ZOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}

//Puts all the accelerometer values in the structure:
void get_accel_xyz(void)
{
	uint8_t tmp_data[7] = {0,0,0,0,0,0};	
	
	//According to the documentation it's X_H, X_L, Y_H, ...
	i2c0_read(IMU_ADDR, IMU_ACCEL_XOUT_H, tmp_data, 7);	
}

// Get gyro X
int16_t get_gyro_x(void)
{
	uint8_t data[2];
	i2c0_read(IMU_ADDR, IMU_GYRO_XOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}

// Get gyro Y
int16_t get_gyro_y(void)
{
	uint8_t data[2];
	i2c0_read(IMU_ADDR, IMU_GYRO_YOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}

// Get gyro Z
int16_t get_gyro_z(void)
{
	uint8_t data[2];
	i2c0_read(IMU_ADDR, IMU_GYRO_ZOUT_H, data, 2);
	return (int16_t)((uint16_t) data[0] << 8) | (data[1]);
}

//Puts all the gyroscope values in the structure:
void get_gyro_xyz(void)
{
	uint8_t tmp_data[7] = {0,0,0,0,0,0,0};
	
	//According to the documentation it's X_H, X_L, Y_H, ...
	i2c0_read(IMU_ADDR, IMU_GYRO_XOUT_H, tmp_data, 6);
}

//// LOW LEVEL FUNCTIONS /////

//write data to an internal register of the IMU.
// you would use this function if you wanted to set configuration values
// for a particular feature of the IMU.
// uint8_t internal_reg_addr: internal register address of the IMU
// uint8_t* pData: pointer to the data we want to send to that address
// uint16_t_t Size: amount of bytes of data pointed to by pData

//ToDo: might wanna move this to the new i2c.c/h files
int imu_write(uint8_t internal_reg_addr, uint8_t* pData, uint16_t length) 
{
	int i = 0;
	uint8_t stat = 0;
	
	i2c_tmp_buf[0] = internal_reg_addr;
	for(i = 1; i < length + 1; i++)
	{
		i2c_tmp_buf[i] = pData[i-1];
	}
	
	//Try to write it up to 5 times
	for(i = 0; i < 5; i++)
	{
		stat = I2C_0_MasterWriteBuf(IMU_ADDR, (uint8_t *) i2c_tmp_buf, length + 1, I2C_0_MODE_COMPLETE_XFER);
		
		if(stat == I2C_0_MSTR_NO_ERROR)
		{
			break;
		}
		/*
		else
		{
			//Release bus:
             I2C_0_BUS_RELEASE;
		}
		*/
		
		CyDelay(10);
	}

	return 0;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

//Copy of the test code used in main.c to test the hardware:
void imu_test_code_blocking(void)
{
	//IMU test code
	
	uint8_t ledg_state = 0;
	//int16_t imu_accel_x = 0;

	/*
	//Single channel test:
	while(1)
	{
		imu_accel_x = get_accel_x();
		send_usb_int16_t(imu_accel_x);
		
		ledg_state ^= 1;
		LED_G_Write(ledg_state);
		
		CyDelay(75);		
	}
	*/
	
	// 3 channels test (only one displayed)
	while(1)
	{
		get_accel_xyz();
		//get_gyro_xyz();
		#ifdef USE_USB
		send_usb_int16_t(imu.gyro.z);
		#endif	//USE_USB
		
		ledg_state ^= 1;
		LED_G_Write(ledg_state);
		
		CyDelay(75);		
	}
}
