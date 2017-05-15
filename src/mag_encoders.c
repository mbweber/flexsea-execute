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
	[This file] mag_encoders: AS504x Magnetic Encoders
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "i2c.h"
#include "mag_encoders.h"
#include "user-ex.h"
#include "filters.h"
#include "flexsea_global_structs.h"
#include "dynamic_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

int32 angtimer_read = 65000;

struct as504x_s as5047, as5048b;

//Magnetic encoder, AS5047:
uint16 spidata_mosi[WORDS_IN_FRAME] = {0,0,0,0,0,0,0};
uint16 spidata_miso[WORDS_IN_FRAME] = {0,0,0,0,0,0,0};
uint16 spidata_mosi2[WORDS_IN_FRAME];
uint16 as5047_angle = 0;
volatile uint8_t spi_isr_state = 0;
volatile uint16 as5047_empty_read = 0;
int32_t spi_read_flag = 0;

//Magnetic encoder, AS5048B:
uint8_t as5048b_bytes[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t as5048b_agc = 0, as5048b_diag = 0;
uint16 as5048b_mag = 0, as5048b_angle = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	

static uint16 add_even_parity_msb(uint16 word);

//****************************************************************************
// Function(s)
//****************************************************************************

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
	#else
		
	(void)reg;
	
	#endif	//USE_AS5047

	return 0;
}

//Get latest readings from the AS5048B position sensor
void get_as5048b_position(void) 
{
	i2c0_read(I2C_ADDR_AS5048B, AD5048B_REG_ANGLE_H, as5048b_bytes, 2);
}

void init_angle_timer(void)
{
    Timer_angleread_Start();
}

//update the number of counts since the last time the angle sensor was read
void update_counts_since_last_ang_read(struct as504x_s *as504x)
{
    angtimer_read = Timer_angleread_ReadCounter(); 
    
    if (angtimer_read>as504x->last_angtimer_read+20000)
    {
        as504x->counts_since_last_ang_read =  as504x->counts_since_last_ang_read + as504x->last_angtimer_read+65000-angtimer_read;
    }
    else
    {
        as504x->counts_since_last_ang_read = as504x->counts_since_last_ang_read + as504x->last_angtimer_read-angtimer_read;
    }
    as504x->last_angtimer_read = angtimer_read;
}

//resent the angle read counter and store the the counts between last two readings
void reset_ang_counter(struct as504x_s *as504x)
{
    update_counts_since_last_ang_read(as504x);
    as504x->last_ang_read_period = as504x->counts_since_last_ang_read-0;
    as504x->counts_since_last_ang_read = 0;
}

//Initialize encoder structures
void init_as504x(struct as504x_s *as504x)
{
	init_diffarr(&as504x->raw_angs_clks);
    //init_diffarr(&as504x->raw_angs_clks_slow);
    init_diffarr(&as504x->raw_vels_cpms);    
    
    as504x->filt_vel_cpms = 0; 
    as504x->signed_ang = 0;
    as504x->signed_ang_vel = 0;
    as504x->ang_abs_clks = 0; 
    as504x->ang_comp_clks = 0;
    as504x->ang_comp_clks_for_cur = 0;
    as504x->num_rot = 0;     
    
    init_angsense(&as504x->raw);
    init_angsense(&as504x->filt);
    as504x->samplefreq = 10000;
    as504x->last_angtimer_read = 0;
    as504x->counts_since_last_ang_read = 0;
    as504x->last_ang_read_period = 0;
}

void init_angsense(struct angsense_s *as)
{
    int i;
    for(i = 0; i < 11; i++)
		as->angs_clks[i] = 0;
        
    as->vels_cpms[0] = 0;
    as->vels_cpms[1] = 0;
    as->vels_ctrl_cpms[0] = 0;
    as->vels_ctrl_cpms[1] = 0;
    
    as->ang_clks = 0;
    as->ang_deg = 0;
    as->vel_cpms = 0;
    as->vel_ctrl_cpms = 0;
    as ->vel_rpm = 0;
}

//update all of the angle variables
void update_as504x(int32_t ang, struct as504x_s *as504x)
{      
    static int64_t raw_ang, raw_vel, raw_ctrl_vel;
    static int64_t avg_ang_sum;
    static int64_t avg_angs[11] = {0,0,0,0,0,0,0,0,0,0,0};
    
    //determine if the encoder has rotated past 0/16383 point and in what direction
    if (ang-as504x->ang_abs_clks<-5000)
    {
        as504x->num_rot++;
    }
    else if ((ang-as504x->ang_abs_clks)>5000)
    {
        as504x->num_rot--;
    }
    
    //assign latest raw angle value
    as504x->ang_abs_clks = ang;
    
    raw_ang = (as504x->num_rot<<14)+as504x->ang_abs_clks; 
    
    //shift the ang values accept the last two, which will be shifted in the filter function
    static int ii;
    avg_ang_sum  = 0;
    as504x->raw.angs_clks[10] = as504x->raw.angs_clks[9];
    as504x->raw.angs_clks[9] = as504x->raw.angs_clks[8];
    as504x->raw.angs_clks[8] = as504x->raw.angs_clks[7];
    
    for (ii=7;ii>1;ii--)
    {
        as504x->raw.angs_clks[ii] = as504x->raw.angs_clks[ii-1];
        avg_ang_sum+=as504x->raw.angs_clks[ii];
    }
    avg_ang_sum+=as504x->raw.angs_clks[0];
    avg_ang_sum+=raw_ang;
    
    for (ii=10;ii>0;ii--) {avg_angs[ii] = avg_angs[ii-1];}
    avg_angs[0] = avg_ang_sum>>3;
    
    raw_vel = (avg_angs[0]-avg_angs[10])*2;
    
    //calculate the simple difference velocity
    //raw_vel = (raw_ang-as504x->raw.angs_clks[10]); //clicks per ms //has to be raw_ang since raw.angs_clks[0] has not been updated yet. 
    raw_ctrl_vel = as504x->raw.vels_cpms[0]*10;
    
   
    if (as504x->samplefreq == 10000)
    {
        as504x->filt.vel_cpms = filt_array_10khz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,5,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_10khz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_10khz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }
    else if (as504x->samplefreq == 250)
    {
        as504x->filt.vel_cpms = filt_array_250hz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,20,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_250hz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_250hz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }
    else if (as504x->samplefreq == 1000)
    {
        as504x->filt.vel_cpms = filt_array_1khz(as504x->raw.vels_cpms, as504x->filt.vels_cpms,20,raw_vel);
        as504x->filt.vel_ctrl_cpms = filt_array_1khz(as504x->raw.vels_ctrl_cpms, as504x->filt.vels_ctrl_cpms,10,raw_ctrl_vel);
        as504x->filt.ang_clks = filt_array_1khz(as504x->raw.angs_clks, as504x->filt.angs_clks,20,raw_ang);
    }    
    
    //calculate the derived angular terms
    as504x->raw.ang_clks = as504x->raw.angs_clks[0];
    as504x->raw.ang_deg = (as504x->raw.ang_clks*360)>>14;
    as504x->raw.vel_cpms = as504x->raw.vels_cpms[0];  
    as504x->raw.vel_ctrl_cpms = as504x->raw.vel_cpms;
    as504x->raw.vel_rpm = (as504x->raw.vel_cpms*366)/100;
    as504x->filt.ang_deg = (as504x->filt.ang_clks*360)>>14;
    as504x->filt.vel_rpm = (as504x->filt.vel_cpms*366)/100;   
    
	as504x->signed_ang = -1 * avg_angs[0] * MOTOR_ORIENTATION;
	as504x->signed_ang_vel = -1 * as504x->filt.vel_cpms * MOTOR_ORIENTATION;
    
    //update the 1 MHz counts from the last angle read
    update_counts_since_last_ang_read(as504x);
    
    //Update the velocity compensated angle//as504x->counts_since_last_ang_read+
    as504x->ang_comp_clks = ((((as504x->counts_since_last_ang_read+53)*(as504x->filt.vel_cpms))/1000+(as504x->ang_abs_clks)+16384)%16384); 
}

void update_as504x_absang(int32_t ang, struct as504x_s *as504x)
{    
    as504x->ang_abs_clks = ang; 
    as504x->ang_comp_clks = ((((66)*(as504x->filt_vel_cpms))/1000+(as504x->ang_abs_clks)+16384)%16384);
}

void update_as504x_vel(struct as504x_s *as504x)
{   
    static int64_t last_ang_abs_clks = 0;    
    //determine if the encoder has rotated past 0/16383 point and in what direction
    if (as504x->ang_abs_clks-last_ang_abs_clks<-5000)
    {
        as504x->num_rot++;
    }
    else if ((as504x->ang_abs_clks-last_ang_abs_clks)>5000)
    {
        as504x->num_rot--;
    }
    last_ang_abs_clks = as504x->ang_abs_clks;
    update_diffarr(&as504x->raw_angs_clks,((as504x->num_rot<<14)+as504x->ang_abs_clks),2);

    as504x->filt_vel_cpms = get_vel_1k_5samples(&as504x->raw_angs_clks)/1000;

    as504x->signed_ang =  as504x->raw_angs_clks.curval * MOTOR_ORIENTATION;
	as504x->signed_ang_vel = as504x->filt_vel_cpms * MOTOR_ORIENTATION;

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
