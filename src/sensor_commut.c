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
#include "control.h"
#include "flexsea_user_structs.h"

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

int32_t PWM_A_Value;
int32_t PWM_B_Value;
int32_t PWM_C_Value;

int16 phaseAcoscoms[2048];
int16 phaseBcoscoms[2048]; 
int16 phaseCcoscoms[2048];  

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

int get_cos_profile(double t, double period)
{   
    static double cos_amp = 1024;
    static double fund_freq;

    fund_freq = round(cos_amp*cos(t*6.2832/period));
    
    return fund_freq;//+sin_amp;    
}

//run at 1 kHz
void find_poles(void)
{
	#if(MOTOR_COMMUT == COMMUT_SINE)
		
	static int32 counter = 0, mincomang = MAX_ENC, mincomangindx = -1;
	const uint16 pwmhigh = PWM_AMP-30, pwmlow = PWM_AMP+15, pausetime = 400;	
	static int32 phasecounter = 0;
	static int32 phase = 0, numsteps = NUMPOLES + 5;
    static int32_t angsum = 0;
    static int32_t angsumcntr = 0;    
	
    static int32_t p0v, p1v, p2v;
        
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
                    p0v = pwmhigh;
                    p1v = pwmlow;
                    p2v = pwmlow;
				break;
			case 1:
                    p0v = pwmhigh;
                    p1v = pwmlow;
                    p2v = pwmhigh;
				break;
			case 2:
                    p0v = pwmlow;
                    p1v = pwmlow;
                    p2v = pwmhigh;
				break;
			case 3:
                    p0v = pwmlow;
                    p1v = pwmhigh;
                    p2v = pwmhigh;
				break;
			case 4:
                    p0v = pwmlow;
                    p1v = pwmhigh;
                    p2v = pwmlow;
				break;
			case 5:
                    p0v = pwmhigh;
                    p1v = pwmhigh;
                    p2v = pwmlow;
				break;
		}
        setDmaPwmCompare(p0v, p1v, p2v);
        
        if (counter>pausetime/2)
        {
            angsum += (as5047.raw_angs_clks.curval+32768);
            angsumcntr++;
        }
        
        if (counter == (pausetime-1))
        {
            //temp_anglemap[phasecounter%NUMPOLES] = as5047.ang_abs_clks;
            temp_anglemap[phasecounter%NUMPOLES] = ((angsum+angsumcntr/2)/angsumcntr)%16384;  
            if (temp_anglemap[phasecounter%NUMPOLES]<mincomang)
            {
                mincomang = temp_anglemap[phasecounter%NUMPOLES];
                mincomangindx = phasecounter%NUMPOLES;
                initpole = phasecounter%6;
            }
            angsum = 0;
            angsumcntr = 0;
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
    static int16_t outs1[6] = {0,0,0,0,0,0};
    static int16_t outs2[6] = {0,0,0,0,0,0};
    load_angles_from_eeprom(anglemap, COMMUTATION);
    initpole = anglemap[126];
    
    int16 ii = 3;
    while(ii<=MAX_ENC)
    {
        fill_comm_tables(ii,outs1);
        fill_comm_tables(ii+1,outs2);
        phaseAcoms[ii>>3] = (outs1[0]+outs2[0])/2;
	    phaseBcoms[ii>>3] = (outs1[1]+outs2[1])/2;
	    phaseCcoms[ii>>3] = (outs1[2]+outs2[2])/2;  
        phaseAcoscoms[ii>>3] = (outs1[3]+outs2[3])/2;
	    phaseBcoscoms[ii>>3] = (outs1[4]+outs2[4])/2;
	    phaseCcoscoms[ii>>3] = (outs1[5]+outs2[5])/2;
        ii += 8;
    }   
}

//ang goes from 0 to 16384
void fill_comm_tables(int32 ang, int16_t * outs)
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
    
    outs[0] = (get_sin_profile((rel_ang),six_period));
	outs[1] = (get_sin_profile((rel_ang+period*2),six_period));
	outs[2] = (get_sin_profile((rel_ang+period*4),six_period));    
    outs[3] = (get_cos_profile((rel_ang),six_period));
	outs[4] = (get_cos_profile((rel_ang+period*2),six_period));
	outs[5] = (get_cos_profile((rel_ang+period*4),six_period));

}

//ang is the current angle of the motor, pwm ranges from -1024 to 1024
//ang should be from 0 to 2048

int32_t induc_amp = 0;  
void sensor_sin_commut(int16 ang, int32 pwm)
{
    #if(MOTOR_COMMUT == COMMUT_SINE)
		
    static int32 bat_volt, curr_pwm;
    
    if (findingpoles == 0)
    {
        if (criticalError())
        {
			//All PWM to 0 - Maximum damping
            PWM_A_Value=PWM_AMP;
            PWM_B_Value=PWM_AMP;
            PWM_C_Value=PWM_AMP;
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
            //at 0 pwm = 1980/2000
            //at 990 pwm = 0/2000

            PWM_A_Value=((((int32)(phaseAcoms[ang])*pwm)+induc_amp*(int32_t)phaseAcoscoms[ang])/1024+PWM_AMP);
            PWM_B_Value=((((int32)(phaseBcoms[ang])*pwm)+induc_amp*(int32_t)phaseBcoscoms[ang])/1024+PWM_AMP);
            PWM_C_Value=((((int32)(phaseCcoms[ang])*pwm)+induc_amp*(int32_t)phaseCcoscoms[ang])/1024+PWM_AMP);
            
        }		
        
        if (PWM_A_Value>989) {PWM_A_Value = 989;}
        else if (PWM_A_Value<1) {PWM_A_Value = 1;}
        
        if (PWM_B_Value>989) {PWM_B_Value = 989;}
        else if (PWM_B_Value<1) {PWM_B_Value = 1;}
        
        if (PWM_C_Value>989) {PWM_C_Value = 989;}
        else if (PWM_C_Value<1) {PWM_C_Value = 1;}

		setDmaPwmCompare((uint16)PWM_A_Value, (uint16)PWM_B_Value, (uint16)PWM_C_Value); 
    }    
	
	#else
		
	(void)ang;
	(void)pwm;
		
	#endif
}

//****************************************************************************
// Private Function(s)
//****************************************************************************


void calc_motor_L(void)
{  
    static struct diffarr_s currs;
    static int32_t mot_induc;
    
    update_diffarr(&currs,ctrl.current.actual_vals.avg,20);
    update_diffarr_avg(&currs,10);
    if (ctrl.active_ctrl == CTRL_NONE)
    {
        mot_induc = 0;
    }
    else
    {
        mot_induc = 70;
    }
    globvar[0] = as5047.signed_ang_vel;
    globvar[1] = currs.avg;
    induc_amp = ((mot_induc*(as5047.signed_ang_vel)*currs.avg)/safety_cop.v_vb_mv)/149;
}
    
