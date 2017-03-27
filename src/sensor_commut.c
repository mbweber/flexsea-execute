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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] JF Duval, Jake Mooney
*****************************************************************************
	[This file] sensor_commut: Angle Sensor Motor Commutation
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "sensor_commut.h"
#include "mag_encoders.h"
#include "safety.h"
#include "mem_angle.h"
#include "motor.h"
#include "user-ex.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

int findingpoles = 0;
uint16 initpole = 1;
uint16 anglemap[128];
uint16 temp_anglemap[128];
uint8_t measure_motor_resistance = 0;
int i2t_flag = 0;

int16_t phaseAcoms[2048];
int16_t phaseBcoms[2048]; 
int16_t phaseCcoms[2048];

uint16 PWM_A_Value;
uint16 PWM_B_Value;
uint16 PWM_C_Value;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	


//****************************************************************************
// Public Function(s)
//****************************************************************************

int get_sin_profile(double t, double period)
{   
    static double sin_amp = PWM_AMP;
    static double fund_freq;

    fund_freq = round(sin_amp*sin(t*6.2832/period));
    
    return fund_freq;//+sin_amp;
    
    //Use the below if you would like 3rd order harmonic injection, but the current signal will become very noisy
    //static double third_freq;
    //static double sum_freq;
    //third_freq = sin_amp/6.0*sin(3*t*6.2832/period);    
    //sum_freq = (fund_freq+third_freq)/0.8662;
    //return (int)sum_freq+sin_amp;
}

//run at 1 kHz
void find_poles(void)
{
	#if(MOTOR_COMMUT == COMMUT_SINE)
		
	static int32 counter = 0, mincomang = MAX_ENC, mincomangindx = -1;
	const uint16 pwmhigh = PWM_AMP-100, pwmlow = PWM_AMP, pausetime = 400;	
	static int32 phasecounter = 0;
	static int32 phase = 0, numsteps = NUMPOLES + 5;
	
	counter++;
	if(counter >= pausetime)
	{ 
        counter = 0;
		phasecounter++;    
	}

	phase = phasecounter%6;
    
    if (phasecounter <= numsteps)
    {
		findingpoles = 1;
		
		switch (phase)
		{
			case 0:
				setDmaPwmCompare(pwmhigh, pwmlow, pwmlow);
				break;
			case 1:
				setDmaPwmCompare(pwmhigh, pwmlow, pwmhigh);
				break;
			case 2:
				setDmaPwmCompare(pwmlow, pwmlow, pwmhigh);
				break;
			case 3:
				setDmaPwmCompare(pwmlow, pwmhigh, pwmhigh);
				break;
			case 4:
				setDmaPwmCompare(pwmlow, pwmhigh, pwmlow);
				break;
			case 5:
				setDmaPwmCompare(pwmhigh, pwmhigh, pwmlow);
				break;
		}
        
        if (counter == (pausetime-1))
        {
            temp_anglemap[phasecounter%NUMPOLES] = as5047.ang_abs_clks;  
            if (as5047.ang_abs_clks<mincomang)
            {
                mincomang = as5047.ang_abs_clks;
                mincomangindx = phasecounter%NUMPOLES;
                initpole = phasecounter%6;
            }
        }
    }
    else 
    {
        if (findingpoles == 1)
        {
            setDmaPwmCompare(PWM_MAX,PWM_MAX,PWM_MAX);
            
            int ii = 0;
            while(mincomangindx<=(NUMPOLES-1))
            {
                anglemap[ii] = temp_anglemap[mincomangindx];
                ii++;
                mincomangindx++;
            }
            
            while (ii<=(NUMPOLES-1))
            {
                anglemap[ii] = temp_anglemap[mincomangindx-NUMPOLES];
                ii++;
                mincomangindx++;
            }
                
            anglemap[126] = initpole;
            anglemap[127] = 1;
            
			#ifdef USE_EEPROM
			
            save_angles_to_eeprom(anglemap, COMMUTATION);
            load_eeprom_to_angles();     
			
			#endif
			
			#ifndef FINDPOLES
			phasecounter = 0;
			phase = 0;
			counter = 0;
			mincomang = MAX_ENC;
			mincomangindx = -1;
			#endif // FINDPOLES 
			
			findingpoles = 0;
        }
    }
	#endif
}

void load_eeprom_to_angles(void)
{
    load_angles_from_eeprom(anglemap, COMMUTATION);
    initpole = anglemap[126];
    
    int16 ii = 0;
    while(ii<=MAX_ENC)
    {
        fill_comm_tables(ii);
        ii = ii+8;
    }   
}

//ang goes from 0 to 16384
void fill_comm_tables(int32 ang)
{
    volatile int32 period = 0, rel_ang;
    int32 six_period = 0;
    int exit_flag = 0;
    
    int32 indx = 0; 
    int32 shiftmap = (initpole)%6;
    
   
    if (ang<anglemap[0])
    {
        period = (anglemap[0]+(MAX_ENC+1)-anglemap[NUMPOLES-1]);
        rel_ang = ang+(MAX_ENC+1)-anglemap[NUMPOLES-1]+period*((NUMPOLES-1+shiftmap)%6);   
    }
    else if (ang<anglemap[1])
    {
        period = anglemap[1]-anglemap[0];
        rel_ang = ang-anglemap[0]+period*(shiftmap%6);
    }
    else if (ang>=anglemap[NUMPOLES-1])
    {
        period = anglemap[0]+(MAX_ENC+1)-anglemap[NUMPOLES-1];
        rel_ang = ang-anglemap[NUMPOLES-1]+period*((NUMPOLES-1+shiftmap)%6);
    }
    else if (ang>=anglemap[NUMPOLES-2])
    {
        period = anglemap[NUMPOLES-1]-anglemap[NUMPOLES-2];
        rel_ang = ang-anglemap[NUMPOLES-2]+period*((NUMPOLES-2+shiftmap)%6);
    }
    else
    {
        indx = ang/(MAX_ENC/NUMPOLES); //the beginnning of the zone
        
        while(exit_flag == 0)
        {
            if (ang<anglemap[indx])
            {
                indx--;
            }
            else if (ang>= anglemap[indx+1])
            {
                indx++;
            }
            else
            {exit_flag = 1;}
        }
        exit_flag = 0;
        period = anglemap[indx+1]-anglemap[indx];
        rel_ang = ang-anglemap[indx]+period*((indx+shiftmap)%6);
        
    }
    six_period = period*6;
    
	phaseAcoms[ang>>3] = (get_sin_profile((rel_ang),six_period));
	phaseBcoms[ang>>3] = (get_sin_profile((rel_ang+period*2),six_period));
	phaseCcoms[ang>>3] = (get_sin_profile((rel_ang+period*4),six_period));
    
}

//ang is the current angle of the motor, pwm ranges from -1024 to 1024
//ang should be from 0 to 2048

void sensor_sin_commut(int16 ang, int32 pwm)
{
    #if(MOTOR_COMMUT == COMMUT_SINE)
		
    static int32 bat_volt, curr_pwm;
    
    if (findingpoles == 0)
    {
        if (criticalError())
        {
			//All PWM to 0 - Maximum damping
            PWM_A_Value=PWM_MAX;
            PWM_B_Value=PWM_MAX;
            PWM_C_Value=PWM_MAX;
        }
        else if (measure_motor_resistance)
        {   
            bat_volt = ((16*safety_cop.v_vb/3+302)*33)>>7; //battery voltage in mV
            curr_pwm = (bat_volt*bat_volt)/979 - ((bat_volt*170)>>8) + 190;
            
            if (curr_pwm > 150)
            curr_pwm = 150;
            else if (curr_pwm<0)
            curr_pwm = 0;
            
            PWM_A_Value=PWM_AMP+curr_pwm;
            PWM_B_Value=PWM_AMP;
            PWM_C_Value=PWM_AMP;
        }
        else
        {
			//Normal operation
            //at 2000 pwm = 1970/2000
            //at 30 pwm = 1/2000

            if (pwm >= 0)
            {
                PWM_A_Value=((uint16)(((int32)(-phaseAcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
                PWM_B_Value=((uint16)(((int32)(-phaseBcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
                PWM_C_Value=((uint16)(((int32)(-phaseCcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
            }
            else
            {
                pwm = -pwm;
                PWM_A_Value=((uint16)(((int32)(phaseAcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
                PWM_B_Value=((uint16)(((int32)(phaseBcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
                PWM_C_Value=((uint16)(((int32)(phaseCcoms[ang])*pwm+512)>>10)+PWM_OFFSET);
            }
            
        }
		
		setDmaPwmCompare(PWM_A_Value, PWM_B_Value, PWM_C_Value); 
        //EX3_Write(0);
    }    
	
	#else
		
	(void)ang;
	(void)pwm;
		
	#endif
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
