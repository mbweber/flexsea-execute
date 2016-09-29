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
	[Contributors] Luke Mooney
*****************************************************************************
	[This file] exo: ExoBoot functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/
	
#ifndef INC_EXO_H
#define INC_EXO_H

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

void init_exo(void);
void exo_fsm(void);


//****************************************************************************
// Definition(s):
//****************************************************************************

#define scale 			1000 	//torque inverse scaling factor
#define momarm 			30 		//lever arm length
#define drivepulley 	14 		//number of teeth on motor pulley
#define spoolpulley		44 		//number of teeth on spool pulley
#define spoolrad		4 		//radius (mm) of spool
#define trans 			(momarm*spoolpulley/drivepulley/spoolrad)	//total transmission ratio

#define percTotalPower	30 		//percentage of biological power to add to ankle during PF
#define powinsert		(1200*scale*percTotalPower/100) 	//Amplitude of parabolic power spike during PF
#define clkang			(2000*trans/360) 	//motor clicks per ankle angle, 2000 clicks/rot / 360 deg * transmission ratio
#define bluetooth		-1 		//-1 is not send commands over bluetooth, must be set to 1 to start sending values
#define subjMass		80 		//subject mass in kg

#define refMaxPFpower	(subjMass*3*percTotalPower/100) 	//reference maximum power during PF from Winter
#define maxDF			(5*clkang) 			//Maximum dorsiflexion allowed by the string (automatically updates and changes)
#define Bv				getvalue(_volts,2) 	//battery volts x10 so 480 = 48 volts	
	//Note: this one won't work (can't call a function in a definition). Use a variable, or fixed arguments.
#define torque			0 					//Instantiate torque
#define speedcoef		100 				//Percentage of voltage to compensate for back EMF of motor
#define effic 			65 					//100xefficiency of total actuator

//Numerical filters, Lowpass Butterworth 6Hz cutoff at 250Hz sampling
#define FILT_PARAM_SIZE	3		//Number of filter coefficients
#define FILT_A0			10000	//x10000
#define FILT_A1			-17874	//x10000
#define FILT_A2			8079	//x10000
#define	FILT_B0			5129 	//x1000000
#define	FILT_B1			10259 	//x1000000
#define	FILT_B2			5129 	//x1000000

//****************************************************************************
// Structure(s)
//****************************************************************************	

#endif	//INC_EXO_H
