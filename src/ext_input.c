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
	[This file] ext_input: External Input Devices/Sensors
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "ext_input.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Encoders:
struct enc_s encoder;
struct as504x_s as5047, as5048b;

//Magnetic encoder, AS5047:
uint16 spidata_mosi[WORDS_IN_FRAME] = {0,0,0,0,0,0,0};
uint16 spidata_miso[WORDS_IN_FRAME] = {0,0,0,0,0,0,0};
uint16 spidata_mosi2[WORDS_IN_FRAME];
uint8 spistatus = 0;
uint16 angleunc = 0;
uint16 as5047_angle = 0;
volatile uint8 spi_isr_state = 0;
volatile uint16 as5047_empty_read = 0;

//Magnetic encoder, AS5048B:
uint8 as5048b_bytes[10] = {0,0,0,0,0,0,0,0,0,0};
uint8 as5048b_agc = 0, as5048b_diag = 0;
uint16 as5048b_mag = 0, as5048b_angle = 0;

//6-ch Strain Amplifier:
uint16 ext_strain[6] = {0,0,0,0,0,0};
uint8 ext_strain_bytes[12];

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	

static uint16 add_even_parity_msb(uint16 word);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialize the quadrature encoder input (QEI) 
void init_qei(void)
{
	#ifdef USE_QEI
	QuadDec_1_Start();
	QuadDec_1_Enable();
	QuadDec_1_SetCounter(QUAD1_INIT);
	#endif
}

//Updates the structure with the latest encoder value
//Only deals with the Controller encoder (no commutation)

int32 refresh_enc_control(void)
{
    //Count: actual, last, difference
	encoder.count_last = encoder.count;
	
	#if(ENC_CONTROL == ENC_QUADRATURE)
		encoder.count = CTRL_ENC_FCT(QuadDec_1_GetCounter());
	#elif(ENC_CONTROL == ENC_ANALOG)
		encoder.count = get_analog_pos();	
	#elif(ENC_CONTROL == ENC_AS5047)
		ctrl.impedance.actual_val = *exec1.enc_ang;
		ctrl.impedance.actual_vel = *exec1.enc_ang_vel;
		ctrl.position.pos = *exec1.enc_ang;
		return *(exec1.enc_ang);
	#elif(ENC_CONTROL == ENC_AS5048B)
		ctrl.impedance.actual_val = *(exec1.enc_ang);
		ctrl.impedance.actual_vel = *(exec1.enc_ang_vel);
		ctrl.position.pos = *(exec1.enc_ang);
		return *(exec1.enc_ang);
    #elif(ENC_CONTROL == ENC_CUSTOM)
		encoder.count = CTRL_ENC_FCT(get_enc_custom());
	#endif
		
	encoder.count_dif = encoder.count - encoder.count_last;
	
	//For the position & impedance controllers we use the last count
	ctrl.position.pos = encoder.count;
	ctrl.impedance.actual_val = encoder.count;
	
	return encoder.count;
}

//Warning: encoder.count seems to be interpreted as a uint... casting (int32) before using it works.

void qei_write(int32 enc)
{
	#ifdef USE_QEI
		//encoder.count = enc;
		QuadDec_1_SetCounter(enc);
		//Note: the read uses CTRL_ENC_FCT(), be careful about what you write!
	#else
		(void)enc;
	#endif
}

int32 qei_read(void)
{
	int32 retval = 0;
	
	#ifdef USE_QEI
		retval = QuadDec_1_GetCounter();
	#endif
	
	return retval;
}

void init_as5047(void)
{
	#ifdef USE_AS5047
		
	//Init SPIM module:
    SPIM_1_Start();
	
	//Word used to read:
	as5047_empty_read = add_even_parity_msb(AS5047_READ | AS5047_REG_NOP);
	
	//Interrupts:
	SPIM_1_SetTxInterruptMode(SPIM_1_INT_ON_BYTE_COMP);
	
	//Interrupt (TX):
	isr_spi_tx_Start();
	#endif	//USE_AS5047
}

uint16 as5047_read_single_isr(uint16 reg)
{
	#ifdef USE_AS5047
		
	//Prepare TX:
	spidata_mosi2[0] = add_even_parity_msb(AS5047_READ | reg);	//1st byte (reg addr)
	spi_isr_state = 0;
	SPIM_1_WriteTxData(spidata_mosi2[0]);	
	
	//(Rest done via ISR)
	#endif //USE_AS5047

	return 0;
}

//Reassembles the bytes we read in words
void strain_6ch_bytes_to_words(uint8 *buf)
{
	ext_strain[0] = ((((uint16)buf[0] << 8) & 0xFF00) | (uint16)buf[1]);
	ext_strain[1] = ((((uint16)buf[2] << 8) & 0xFF00) | (uint16)buf[3]);
	ext_strain[2] = ((((uint16)buf[4] << 8) & 0xFF00) | (uint16)buf[5]);
	ext_strain[3] = ((((uint16)buf[6] << 8) & 0xFF00) | (uint16)buf[7]);
	ext_strain[4] = ((((uint16)buf[8] << 8) & 0xFF00) | (uint16)buf[9]);
	ext_strain[5] = ((((uint16)buf[10] << 8) & 0xFF00) | (uint16)buf[11]);
}

//Get latest readings from the 6-ch strain sensor. Using the Compressed version,
//9bytes, 12-bits per sensor
void get_6ch_strain(void) 
{	
	i2c0_read(I2C_SLAVE_ADDR_6CH, MEM_R_CH1_H, ext_strain_bytes, 9);
}

//Get latest readings from the AS5048B position sensor
void get_as5048b_position(void) 
{
	i2c0_read(I2C_ADDR_AS5048B, AD5048B_REG_ANGLE_H, as5048b_bytes, 2);
}

//Converts from ADC reading to position
int16 get_analog_pos(void)
{
	#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
		return CTRL_ENC_FCT(adc1_res_filtered[0]);
	#else
		return 0;
	#endif
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//When writing to the AS5047, MSB has to be an Even parity bit
static uint16 add_even_parity_msb(uint16 word)
{
	uint16 ret = 0;
	
	//__builtin_parity() returns 0 for even, 1 for odd
	// MSB = 1 when it's odd
	if(__builtin_parity((int)word))
	{
		ret = (word | PARITY_1);
	}
	else
	{
		ret = (word & PARITY_0);
	}
	
	return ret;
}

//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

void as5047_test_code_blocking(void)
{
	//Test code isn't compatible with new ISR-based function.
	//Write a new version if needed.
}

void strain_amp_6ch_test_code_blocking(void)
{
	while(1)
	{
		i2c0_read(I2C_SLAVE_ADDR_6CH, MEM_R_CH1_H, ext_strain_bytes, 12);
		strain_6ch_bytes_to_words(ext_strain_bytes);
		CyDelay(100);
	}
}

void as5048b_test_code_blocking(void)
{
	while(1)
	{
		i2c0_read(I2C_ADDR_AS5048B, AD5048B_REG_AGC, as5048b_bytes, 6);
		
		as5048b_agc = as5048b_bytes[0];
		as5048b_diag = as5048b_bytes[1] & 0x0F;
		as5048b_mag = (as5048b_bytes[2]<<6) + (as5048b_bytes[3]&0x3F); 
		as5048b_angle = (as5048b_bytes[4]<<6) + (as5048b_bytes[5]&0x3F);
		
		CyDelay(100);
	}
}
