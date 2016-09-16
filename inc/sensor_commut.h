//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@media.mit.edu
// 02/2016
//****************************************************************************
// sensor_commut: Angle Sensor Motor Commutation
//****************************************************************************
	
#ifndef INC_SENSOR_COMMUT_H
#define INC_SENSOR_COMMUT_H

//****************************************************************************
// Include(s)
//****************************************************************************		
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern int findingpoles;
extern int8 phase_a_with_pos_current[2048]; 
extern int8 phase_b_with_pos_current[2048]; 
extern int8 phase_c_with_pos_current[2048]; 
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************	

void motor_spi_block_commutation(int angle);
void motor_spi_block_commutation_triangletest();
void motor_spi_findpoles();
void sensor_commut_1(void);
void find_poles(void);
void find_poles_blocking(void);
void find_poles2(void);
void non_blocking_step_test(void);
void non_blocking_sin_commut(int16 pwm);
void load_eeprom_to_angles(void);
void fill_comm_tables(int32);
void sensor_sin_commut(int16, int32);

void test_sinusoidal_blocking(void);
int get_sin_profile(int32, int32);

//****************************************************************************
// Definition(s):
#define NUMPOLES        126
#define SHIFT_G         0
#define PWM_DEAD        90 //dead time causes by the PWM module
#define MAX_ENC         16383
//#define PWM_DEAD2       41 //dead time caused by the opening and closing of the FETS
//****************************************************************************	

//****************************************************************************
// Structure(s)
//****************************************************************************	

	
#endif	//INC_SENSOR_COMMUT_H
	