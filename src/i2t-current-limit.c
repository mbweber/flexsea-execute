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
	[This file] I2t current limit
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-04 | jfduval | New file, copied from Battery
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "i2t-current-limit.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Returns the square of a uint8 value
const uint16 squared[256] = {0,1,4,9,16,25,36,49,64,81,100,121,144,
	169,196,225,256,289,324,361,400,441,484,529,576,625,
	676,729,784,841,900,961,1024,1089,1156,1225,1296,1369,1444,
	1521,1600,1681,1764,1849,1936,2025,2116,2209,2304,2401,2500,2601,
	2704,2809,2916,3025,3136,3249,3364,3481,3600,3721,3844,3969,4096,
	4225,4356,4489,4624,4761,4900,5041,5184,5329,5476,5625,5776,5929,
	6084,6241,6400,6561,6724,6889,7056,7225,7396,7569,7744,7921,8100,
	8281,8464,8649,8836,9025,9216,9409,9604,9801,10000,10201,10404,10609,
	10816,11025,11236,11449,11664,11881,12100,12321,12544,12769,12996,13225,13456,
	13689,13924,14161,14400,14641,14884,15129,15376,15625,15876,16129,16384,16641,
	16900,17161,17424,17689,17956,18225,18496,18769,19044,19321,19600,19881,20164,
	20449,20736,21025,21316,21609,21904,22201,22500,22801,23104,23409,23716,24025,
	24336,24649,24964,25281,25600,25921,26244,26569,26896,27225,27556,27889,28224,
	28561,28900,29241,29584,29929,30276,30625,30976,31329,31684,32041,32400,32761,
	33124,33489,33856,34225,34596,34969,35344,35721,36100,36481,36864,37249,37636,
	38025,38416,38809,39204,39601,40000,40401,40804,41209,41616,42025,42436,42849,
	43264,43681,44100,44521,44944,45369,45796,46225,46656,47089,47524,47961,48400,
	48841,49284,49729,50176,50625,51076,51529,51984,52441,52900,53361,53824,54289,
	54756,55225,55696,56169,56644,57121,57600,58081,58564,59049,59536,60025,60516,
	61009,61504,62001,62500,63001,63504,64009,64516,65025};
int16 currentSamples[I2T_SAMPLES];
uint8 currentSampleIndex = 0;

//****************************************************************************
// Private Function Prototype(s)
//****************************************************************************

static uint8 getCurrentSampleIndex(void);
static void resetCurrentSampleIndex(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function every ms, and give it the latest current reading
//Use a signed value, without offset
void i2t_sample(int32 lastCurrentRead)
{
	static int tb_div = 1;
	uint8_t index = 0;
	
	//We create a 12ms timebase to get 8 samples per calculation:
	tb_div++;
	tb_div %= 12;
	if(!tb_div)
	{
		//Ready for a new sample:
		index = getCurrentSampleIndex();
		if(index < I2T_SAMPLES)
		{
			//Only save I2T_SAMPLES, values, if there is more we discard.
			currentSamples[index] = lastCurrentRead;
		}
	}
}

//Call this function every 100ms. It will use the last samples saved by i2t_sample()
//Return: RET_I2T_[NORMAL, WARNING, LIMIT]
int i2t_compute(void)
{
	int i = 0;
	int sampleAverage = 0;
	uint16_t squaredCurrent = 0;
	static uint32_t integral = 0;
	
	//Average samples:
	for(i = 0; i < I2T_SAMPLES; i++)
	{
		//We use absolute values:
		sampleAverage += abs(currentSamples[i]);
	}
	sampleAverage = (sampleAverage >> I2T_SAMPLES_SHIFT);
	resetCurrentSampleIndex();	//Ready for next cycle
	
	//Now we need to bring that value down to 8bits-ish, and saturate it:
	sampleAverage = sampleAverage >> I2C_SCALE_DOWN_SHIFT;
	if(sampleAverage > 255)
	{
		sampleAverage = 255;
	}
	
	//We are now ready to square the value and add it to our integral:
	squaredCurrent = squared[sampleAverage];
	integral = integral + squaredCurrent;
	if(integral > I2T_LEAK)
	{
		integral -= I2T_LEAK;
	}
	else
	{
		integral = 0;
	}
	
	//Last step: limits
	if(integral < I2T_WARNING)
	{
		//Lower than the warning treshold
		return RET_I2T_NORMAL;
	}
	else
	{
		if(integral < I2T_LIMIT)
		{
			//Warning - getting close to the limit
			return RET_I2T_WARNING;
		}
		else
		{
			//Problems!
			return RET_I2T_LIMIT;
		}
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static uint8_t getCurrentSampleIndex(void)
{
	currentSampleIndex++;
	return currentSampleIndex-1;
}

static void resetCurrentSampleIndex(void)
{
	currentSampleIndex = 0;
}
