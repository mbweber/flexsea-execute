//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval, Luke Mooney & Jake Mooney
// jfduval@media.mit.edu, lmooney@mit.edu, mooneyj@mit.edu
// 05/2016
//****************************************************************************
// sensor_commut: Angle Sensor Motor Commutation
//****************************************************************************

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "sensor_commut.h"

//****************************************************************************
// Variable(s)
//****************************************************************************



int findingpoles = 0;
uint16 anglemap[128] = {17,152,283,418,550,681,816,948,1077,1206,1337,1466,1592,1722,1851,1979,2104,2235,2361,2490,2621,2752,2879,3008,3144,3270,3399,3532,3663,3796,3927,4056,4187,4316,4449,4579,4709,4836,4967,5094,5220,5349,5477,5600,5728,5856,5983,6109,6235,6366,6491,6618,6751,6880,7012,7139,7273,7406,7537,7669,7805,7936,8073,8206,8340,8471,8610,8738,8871,9004,9136,9265,9395,9522,9655,9782,9911,10042,10170,10294,10427,10557,10680,10811,10942,11070,11199,11332,11458,11590,11725,11854,11985,12113,12246,12378,12507,12640,12771,12899,13025,13154,13285,13409,13538,13662,13791,13917,14044,14171,14299,14426,14554,14682,14808,14940,15071,15200,15332,15468,15600,15728,15867,15999,16130,16267,0,0};

uint16 initpole = 1;

uint16 temp_anglemap[128] = {123,262,392,529,668,792,923,1054,1188,1315,1461,1586,\
						1717,1850,1973,2104,2240,2370,2501,2631,2755,2877,3016,\
						3137,3273,3403,3526,3647,3779,3903,4030,4167,4291,4413,\
						4546,4662,4794,4923,5055,5179,5312,5431,5555,5691,5817,\
						5950,6085,6204,6332,6462,6593,6721,6865,6990,7119,7254,\
						7378,7510,7650,7783,7917,8052,8180,8305,8450,8578,8719,\
						8857,8984,9112,9249,9378,9512,9655,9786,9911,10051,10169,\
						10304,10438,10573,10701,10836,10956,11081,11217,11344,\
						11477,11611,11730,11858,11984,12112,12237,12379,12501,\
						12627,12758,12877,13003,13139,13267,13396,13525,13649,\
						13769,13907,14027,14164,14297,14421,14545,14678,14801,\
						14932,15073,15200,15327,15464,15582,15717,15852,15988,\
						16116,16255,16378,0,0};



//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	


//****************************************************************************
// Public Function(s)
//****************************************************************************





//int32 sin_tbl[200] = {0,13,26,39,52,65,78,91,104,117,129,142,155,168,180,193,206,218,231,243,256,268,281,293,305,317,329,341,353,365,377,388,400,411,423,434,445,456,467,478,489,500,510,521,531,541,552,562,572,581,591,601,610,619,629,638,647,655,664,673,681,689,698,706,713,721,729,736,744,751,758,765,772,778,785,791,797,803,809,815,821,826,832,837,842,847,852,857,861,866,870,874,879,882,886,890,893,897,900,903,906,909,912,915,918,920,922,925,927,929,931,933,934,936,937,939,940,941,943,944,945,945,946,947,948,948,949,949,949,950,950,950,950,950,950,950,950,949,949,949,949,948,948,947,947,946,946,945,944,944,943,942,942,941,940,939,938,938,937,936,935,934,933,933,932,931,930,929,929,928,927,926,925,925,924,923,923,922,921,921,920,919,919,918,918,917,917,917,916,916,916,915,915,915,915,914,914,914,914,914};
//int32 sin_tbl[200] = {0,13,27,40,54,67,81,94,107,121,134,147,161,174,187,200,213,226,239,252,265,278,291,304,316,329,341,354,366,378,390,403,415,426,438,450,462,473,485,496,507,518,529,540,551,561,572,582,593,603,613,623,633,642,652,661,670,680,689,697,706,715,723,732,740,748,756,763,771,778,786,793,800,807,814,820,827,833,839,845,851,857,863,868,873,878,884,888,893,898,902,907,911,915,919,923,926,930,933,937,940,943,946,949,951,954,956,959,961,963,965,967,969,970,972,973,975,976,977,978,979,980,981,982,982,983,984,984,984,985,985,985,985,985,985,985,985,984,984,984,984,983,983,982,982,981,980,980,979,978,978,977,976,975,975,974,973,972,971,970,970,969,968,967,966,965,964,964,963,962,961,960,960,959,958,957,957,956,955,955,954,953,953,952,952,951,951,950,950,950,949,949,949,948,948,948,948,948,948,948};
//int32 sin_tbl[200] = {0,7,13,20,27,33,40,46,53,59,66,73,79,86,92,99,105,111,118,124,131,137,143,149,156,162,168,174,180,186,192,198,204,210,216,222,227,233,239,244,250,255,261,266,271,276,282,287,292,297,302,307,311,316,321,326,330,335,339,343,348,352,356,360,364,368,372,376,380,383,387,390,394,397,401,404,407,410,413,416,419,422,425,427,430,433,435,437,440,442,444,446,449,451,452,454,456,458,460,461,463,464,466,467,468,470,471,472,473,474,475,476,477,478,479,479,480,481,481,482,482,483,483,483,484,484,484,484,485,485,485,485,485,485,485,485,485,485,485,484,484,484,484,484,483,483,483,482,482,482,481,481,481,480,480,479,479,479,478,478,477,477,477,476,476,475,475,474,474,474,473,473,472,472,472,471,471,471,470,470,470,469,469,469,469,468,468,468,468,468,467,467,467,467,467,467,467,467,467,467};
int32 sin_tbl[200] = {0,13,26,39,52,65,78,91,104,117,130,143,156,169,181,194,207,220,232,245,257,270,282,294,307,319,331,343,355,367,379,390,402,413,425,436,448,459,470,481,492,502,513,524,534,544,555,565,575,584,594,604,613,623,632,641,650,659,668,676,685,693,701,709,717,725,733,740,748,755,762,769,776,782,789,795,802,808,814,820,825,831,836,842,847,852,857,861,866,870,875,879,883,887,891,895,898,902,905,908,911,914,917,920,922,925,927,929,932,934,936,937,939,941,942,944,945,946,947,949,950,950,951,952,953,953,954,954,954,955,955,955,955,955,955,955,955,954,954,954,954,953,953,952,952,951,951,950,949,949,948,947,946,946,945,944,943,943,942,941,940,939,938,938,937,936,935,934,933,933,932,931,930,930,929,928,927,927,926,925,925,924,924,923,923,922,922,921,921,921,920,920,920,920,919,919,919,919,919,919};

//returns 1910 x SIN(t) where period is the period length
int get_sin_profile(int32 t, int32 period)
{   
    t = t%period;
    
    t = t*800/period; //scaled to be between 0 and 800
    
    if (t<200)
    {
        return (955+sin_tbl[t]);
    }
    else if (t<400)
    {
        return (955+sin_tbl[399-t]);
    }
    else if (t<600)
    {
        return (955-sin_tbl[t-400]);
    }
    else
    {
        return (955-sin_tbl[799-t]);
    }
    

}

uint16 phaseAcoms[2048];
uint16 phaseBcoms[2048]; 
uint16 phaseCcoms[2048];  
int8 phase_a_with_pos_current[2048]; 
int8 phase_b_with_pos_current[2048]; 
int8 phase_c_with_pos_current[2048]; 

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
            
            ii = 0;
            while(ii<=MAX_ENC)
            {
                fill_comm_tables(ii);
                ii = ii+8;
            }
     
            findingpoles = 0;
        }
    }
}

void load_eeprom_to_angles(void)
{
    load_angles_from_eeprom(anglemap);
    
    int16 ii = 0;
    while(ii<=MAX_ENC)
    {
        fill_comm_tables(ii);
        ii = ii+8;
    }   
    initpole = anglemap[126];

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
        shift = SHIFT_G;    
    }
    else if (ang<anglemap[1])
    {
        period = anglemap[1]-anglemap[0];
        rel_ang = ang-anglemap[0]+period*(shiftmap%6);
        shift = SHIFT_G;
    }
    else if (ang>=anglemap[NUMPOLES-1])
    {
        period = anglemap[0]+(MAX_ENC+1)-anglemap[NUMPOLES-1];
        rel_ang = ang-anglemap[NUMPOLES-1]+period*((NUMPOLES-1+shiftmap)%6);
        shift = SHIFT_G;
    }
    else if (ang>=anglemap[NUMPOLES-2])
    {
        period = anglemap[NUMPOLES-1]-anglemap[NUMPOLES-2];
        rel_ang = ang-anglemap[NUMPOLES-2]+period*((NUMPOLES-2+shiftmap)%6);
        shift = SHIFT_G;
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
        shift = SHIFT_G;


    }
    six_period = period*6;
    
	phaseAcoms[ang>>3] = (get_sin_profile((rel_ang+shift),six_period));
	phaseBcoms[ang>>3] = (get_sin_profile((rel_ang+period*2+shift),six_period));
	phaseCcoms[ang>>3] = (get_sin_profile((rel_ang+period*4+shift),six_period));
    
    //determine the sign of the current at a specific angle for each phase
    if (phaseAcoms[ang>>3]>= 985)
        phase_a_with_pos_current[ang>>3] = 1;
    else
        phase_a_with_pos_current[ang>>3] = -1;
        
    if (phaseBcoms[ang>>3]>= 985)
        phase_b_with_pos_current[ang>>3] = 1;
    else
        phase_b_with_pos_current[ang>>3] = -1;
        
    if (phaseCcoms[ang>>3]>= 985)
        phase_c_with_pos_current[ang>>3] = 1;
    else
        phase_c_with_pos_current[ang>>3] = -1;

}

int64 counter = 0;
//ang is the current angle of the motor, pwm ranges from -1024 to 1024
//ang should be from 0 to 2048
void sensor_sin_commut(int16 ang, int32 pwm)
{
    if (findingpoles == 0)
    {
        if (pwm>= 0)
        {
            PWM_A_WriteCompare1((uint16)(((int32)(phaseAcoms[ang])*pwm)>>10)+PWM_DEAD);
            PWM_B_WriteCompare1((uint16)(((int32)(phaseBcoms[ang])*pwm)>>10)+PWM_DEAD);
            PWM_C_WriteCompare1((uint16)(((int32)(phaseCcoms[ang])*pwm)>>10)+PWM_DEAD);
        }
        else
        {
            pwm = -pwm;
            PWM_A_WriteCompare1((uint16)(((int32)(1970-phaseAcoms[ang])*pwm)>>10)+PWM_DEAD);
            PWM_B_WriteCompare1((uint16)(((int32)(1970-phaseBcoms[ang])*pwm)>>10)+PWM_DEAD);
            PWM_C_WriteCompare1((uint16)(((int32)(1970-phaseCcoms[ang])*pwm)>>10)+PWM_DEAD);
        }
              
    }
    
    counter++;
    
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
