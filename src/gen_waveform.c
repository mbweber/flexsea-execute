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
	[This file] gen_waveform: Generate waveforms for some test functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "gen_waveform.h"

//****************************************************************************
// Variable(s)
//****************************************************************************


//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	


//****************************************************************************
// Public Function(s)
//****************************************************************************

uint16 output_sine(void)
{
	static double angle = 0;
	static double ret = 0;
	uint8 output = 0;
	
	//if(t1_new_value == 1)
	{	
		//t1_new_value = 0;
		
		angle += STEP;
		if(angle >= (2*PI))
			angle = 0;
		ret = sin(angle) + 1;
		
		output = (uint8)(ret*127);
		
		//Output on DAC:
		//VDAC8_2_SetValue(output);
		
		CyDelayUs(DELAY);
	}	
	
	return output;
}

uint16 output_arb(void)
{
	static double angle = 0;
	static double ret = 0;
	static uint8 output = 0;
	static int16 i = 0;
	
	i++;
	
	//Slow ramp-up
	if(i < 100)
		output = i;
	
	//Fast ramp-down
	if(i >= 100 && i <= 150)
		output -= 2;
	
	//Castellation
	if(i > 150 && i <= 160)
		output = 255;
	if(i > 160 && i <= 170)
		output = 0;
	if(i > 180 && i <= 190)
		output = 255;
	if(i > 190 && i <= 200)
		output = 0;
	if(i > 200 && i <= 210)
		output = 255;

	if(i > 210)
	{
		angle += STEP;
		if(angle >= (2*PI))
		{
			angle = 0;
			i = 0;
		}
		ret = sin(angle) + 1;		
		output = (uint8)(ret*127);
	}
	
	//Output on DAC:
	//VDAC8_2_SetValue(output);	
	
	CyDelayUs(DELAY);
	
	return output;
}

//Current/thermal test. 5A average, 20A pulses
//100ms every second.
uint16 output_step(void)
{
	static int16 i = 0;
	uint16 output = 0;
	
	i++;
	
	//Slow ramp-up
	if(i < 785)
		output = 0;	//1850;

	if(i >= 785 && i <= 7500)
		output = 73;	//287;
	
	if(i > 7500)
	{
		i = 0;
		output = 100;	//287;
	}
	
	//Output on DAC:
	//VDAC8_2_SetValue(output);	
	
	CyDelayUs(DELAY);
	
	return output;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************


//****************************************************************************
// Test Function(s) - Use with care!
//****************************************************************************

//****************************************************************************
// Deprecated Function(s)
//****************************************************************************

void init_sine_gen(void)
{
	//Add DAC to top design if you want to use that
	//VDAC8_2_Start();
}
