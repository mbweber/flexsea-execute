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

//****************************************************************************
// Variable(s)
//****************************************************************************

int findingpoles = 0;
uint16 initpole = 1;
uint16 anglemap[128];
uint16 temp_anglemap[128];
int i2t_flag = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	


//****************************************************************************
// Public Function(s)
//****************************************************************************

//quarter SIN
/*
int32 sin_tbl[200] = {0,8,15,23,30,38,45,53,60,68,75,83,90,98,105,113,120,128,\
						135,143,150,158,165,172,180,187,195,202,209,217,224,231,\
						239,246,253,260,268,275,282,289,297,304,311,318,325,332,\
						339,346,353,360,367,374,381,388,395,402,409,415,422,429,\
						436,442,449,456,462,469,475,482,488,495,501,508,514,520,\
						527,533,539,545,552,558,564,570,576,582,588,594,600,605,\
						611,617,623,628,634,640,645,651,656,662,667,673,678,683,\
						688,694,699,704,709,714,719,724,729,734,739,743,748,753,\
						757,762,766,771,775,780,784,788,792,797,801,805,809,813,\
						817,821,825,828,832,836,839,843,846,850,853,857,860,863,\
						866,870,873,876,879,882,884,887,890,893,895,898,901,903,\
						905,908,910,912,915,917,919,921,923,925,927,928,930,932,\
						933,935,936,938,939,941,942,943,944,945,946,947,948,949,\
						950,951,951,952,953,953,954,954,954,955,955,955,955,955};
*/

//quarter SIN with 6xFreq current ripple compensation
int32 sin_tbl[200] = {0,7,15,22,29,37,44,52,59,67,75,82,90,97,105,113,121,128,\
                      136,144,152,160,167,175,183,191,199,206,214,222,230,238,\
                      246,253,261,269,277,284,292,300,307,315,322,330,338,345,\
                      352,360,367,375,382,389,396,403,410,417,424,431,438,445,\
                      452,458,465,471,478,484,491,497,503,510,516,522,528,534,\
                      539,545,551,557,562,568,573,579,584,589,595,600,605,610,\
                      615,620,625,630,634,639,644,649,653,658,662,667,671,676,\
                      680,684,688,693,697,701,705,709,713,717,722,726,729,733,\
                      737,741,745,749,753,757,761,764,768,772,776,779,783,787,\
                      791,794,798,802,805,809,812,816,820,823,827,830,834,837,\
                      841,844,847,851,854,858,861,864,867,871,874,877,880,883,\
                      886,889,892,895,898,900,903,906,909,911,914,916,919,921,\
                      923,925,927,930,932,933,935,937,939,940,942,943,945,946,\
                      947,948,949,950,951,952,953,953,954,954,955,955,955,955};

//returns 1910 x SIN(t)+955 where period is the period length
int get_sin_profile(int32 t, int32 period)
{   
    t = t%period;
    
    t = t*800/period; //scaled to be between 0 and 800
    
    int sin_value; 
    
    if (t<200)
    {
        sin_value = (955+sin_tbl[t]);
    }
    else if (t<400)
    {
        sin_value = (955+sin_tbl[399-t]);
    }
    else if (t<600)
    {
        sin_value = (955-sin_tbl[t-400]);
    }
    else
    {
        sin_value = (955-sin_tbl[799-t]);
    }
    
    return sin_value;
}

uint16 phaseAcoms[2048];
uint16 phaseBcoms[2048]; 
uint16 phaseCcoms[2048];   

//run at 1 kHz
void find_poles(void)
{
	static int32 counter = 0, pausetime = 400, mincomang = MAX_ENC, mincomangindx = -1;
	uint16 pwmhigh = 200, pwmlow = 0;	
	static int32 phasecounter = 0;
	static int32 phase = 0, numsteps = 0;
    
    numsteps = NUMPOLES + 5;
	
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
				PWM_A_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_B_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_C_WriteCompare1(pwmlow+PWM_DEAD);
				break;
			case 1:
				PWM_A_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_B_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_C_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				break;
			case 2:
				PWM_A_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_B_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_C_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				break;
			case 3:
				PWM_A_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_B_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_C_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				break;
			case 4:
				PWM_A_WriteCompare1(pwmlow+PWM_DEAD);
				PWM_B_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_C_WriteCompare1(pwmlow+PWM_DEAD);
				break;
			case 5:
				PWM_A_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_B_WriteCompare1((uint16)((int32)pwmhigh)+PWM_DEAD);
				PWM_C_WriteCompare1(pwmlow+PWM_DEAD);
				break;
		}
        
        if (counter == (pausetime-1))
        {
            temp_anglemap[phasecounter%NUMPOLES] = as5047.angle_raws[0];  
            if (as5047.angle_raws[0]<mincomang)
            {
                mincomang = as5047.angle_raws[0];
                mincomangindx = phasecounter%NUMPOLES;
                initpole = phasecounter%6;
            }
        }
    }
    else 
    {
        if (findingpoles == 1)
        {
            PWM_A_WriteCompare1(0);
    	    PWM_B_WriteCompare1(0);
    		PWM_C_WriteCompare1(0);
            
            
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
            
            save_angles_to_eeprom(anglemap);
            load_eeprom_to_angles();     
            findingpoles = 0;
        }
    }
}

void load_eeprom_to_angles(void)
{
    load_angles_from_eeprom(anglemap);
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
    uint16 val = 0;
	int32 pwm_timer = 0, shift = 0, t_shift = 0, tt_shift = 0;
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
    shift = 0;//SHIFT_G;
    six_period = period*6;
    
	phaseAcoms[ang>>3] = (get_sin_profile((rel_ang+shift),six_period));
	phaseBcoms[ang>>3] = (get_sin_profile((rel_ang+period*2+shift),six_period));
	phaseCcoms[ang>>3] = (get_sin_profile((rel_ang+period*4+shift),six_period));
    
}

//ang is the current angle of the motor, pwm ranges from -1024 to 1024
//ang should be from 0 to 2048
void sensor_sin_commut(int16 ang, int32 pwm)
{
    
    static uint16 PWM_A_Value;
    static uint16 PWM_B_Value;
    static uint16 PWM_C_Value;
    
    if (findingpoles == 0)
    {
        if (criticalError())
        {
			//All PWM to 0 - Maximum damping
            PWM_A_Value=0;
            PWM_B_Value=0;
            PWM_C_Value=0;
        }
        else
        {
			//Normal operation
			
            if (pwm >= 0)
            {
                PWM_A_Value=((uint16)(((int32)(phaseAcoms[ang])*pwm)>>10)+PWM_DEAD);
                PWM_B_Value=((uint16)(((int32)(phaseBcoms[ang])*pwm)>>10)+PWM_DEAD);
                PWM_C_Value=((uint16)(((int32)(phaseCcoms[ang])*pwm)>>10)+PWM_DEAD);
            }
            else
            {
                pwm = -pwm;
                PWM_A_Value=((uint16)(((int32)(1970-phaseAcoms[ang])*pwm)>>10)+PWM_DEAD);
                PWM_B_Value=((uint16)(((int32)(1970-phaseBcoms[ang])*pwm)>>10)+PWM_DEAD);
                PWM_C_Value=((uint16)(((int32)(1970-phaseCcoms[ang])*pwm)>>10)+PWM_DEAD);
            }
        }
		
        PWM_A_WriteCompare1(PWM_A_Value);
        PWM_B_WriteCompare1(PWM_B_Value);
        PWM_C_WriteCompare1(PWM_C_Value);              
    }    
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
