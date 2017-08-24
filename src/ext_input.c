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
#include "control.h"
#include "user-ex.h"
#include "analog.h"
#include "flexsea_global_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//QEI encoder:
struct enc_s encoder;

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
		encoder.count_dif = encoder.count - encoder.count_last;
		ctrl.position.pos = encoder.count;
		ctrl.impedance.actual_val = encoder.count;
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

//Converts from ADC reading to position
int16 get_analog_pos(void)
{
	#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
		return CTRL_ENC_FCT(adc1_res_filtered[0]);
	#else
		return 0;
	#endif
}
