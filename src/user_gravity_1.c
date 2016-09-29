//****************************************************************************
// Dephy, Inc.
// Jean-Francois (JF) Duval
// jfduval@dephy.com
// 07/2016
//****************************************************************************
// user_gravity_1: Dephy's Gravity Boot v1 Functions
//****************************************************************************
 
//****************************************************************************
// Include(s)
//****************************************************************************
 
#include "main.h"
#include "user_gravity_1.h"
 
//****************************************************************************
// Variable(s)
//****************************************************************************
int32 dg_state = -1, dg_statetimer = 0, dg_maxDF = 0, dg_act_ang = 0, dg_ank_ang = 0, dg_trans = 300, dg_batvolt = 0, dg_spring_eq = -500;
int32 dg_spr_torq = 0, dg_max_torq = 0, dg_strain_read;

int32 testarray[10] = {0,0,0,0,0,0,0,0,0,0};
//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  
static void change_state(int32);
static void gravity_1_refresh_values(void);
static void dg_get_ank_ang_trans(void);
static void dg_set_ank_torque(int32);
static void dg_set_mot_torque(int32);
static void dg_set_mot_volt(int32);


//****************************************************************************
// Public Function(s)
//****************************************************************************
 
//Call this function once in main.c, just before the while()
void init_gravity_1(void)
{   
   load_eeprom_to_angles();
   ctrl.active_ctrl = CTRL_OPEN;
   dg_state = -1;
}
 
//Finite State Machine.
//Call this function in one of the main while time slots.

//positive PWM results in PF
//as5047.angle_conts[0] increases in PF 
void gravity_1_fsm(void)
{
    
     
    gravity_1_refresh_values();
    
    
    switch (dg_state)
    {
        case -1:
            dg_set_mot_volt(0);
            if (dg_statetimer>200)
            {
                dg_maxDF = as5047.angle_conts[0];
                ctrl.active_ctrl = CTRL_CURRENT;
                ctrl.current.setpoint_val = 0;
                ctrl.current.gain.I_KP = 100;
                ctrl.current.gain.I_KI = 10;
                change_state(0);
            }
            break;
        case 0:
			//dg_set_mot_volt(1500);
            //motor_open_speed_1(100);
            //global_variable_1 = -10*(as5047.angle_conts[0]-dg_maxDF)/12;
            ctrl.current.setpoint_val = (as5047.angle_conts[0]-dg_maxDF)/12;
            break;
        case 1:

            break;
        case 2:
            break;  
        case 3:
            break;
	}
   
    

}



//***ToDo***
#if(ACTIVE_PROJECT == PROJECT_DEPHY_GRAVITY_1)
int32 get_enc_custom()
{
    int32 mt;
	
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	mt = 0;
    #else
	mt = 0;
    #endif
	
    return mt;
}
#endif
 
//****************************************************************************
// Private Function(s)
//****************************************************************************
 
//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.
 
//Here's an example function:
static void gravity_1_refresh_values(void)
{
	dg_statetimer++;
    dg_batvolt = (16*safety_cop.v_vb/3+302)*33; //battery voltage in mV
    dg_act_ang = ((as5047.angle_conts[0]-dg_maxDF)*360)/16384;
    dg_strain_read = strain_read();
    dg_get_ank_ang_trans();
}

static void change_state(int32 s)
{
    dg_state = s;
    dg_statetimer = 0;
}

static void dg_get_ank_ang_trans(void)
{
    /*strut v3
    float a1 = -2.124/1000000;
    float a2 = 0.001729;
    float a3 = -3.325;
    float a4 = 3318;
    */
    
    float a1 = -2.757/1000000;
    float a2 = 0.002415;
    float a3 = -3.877;
    float a4 = 2835;
    

    float dgaa;
    dgaa = dg_act_ang; //actuator angle is in degrees
    
    dg_ank_ang = (int32)((float)(a1*dgaa*dgaa*dgaa+a2*dgaa*dgaa+a3*dgaa+a4));//100x the ankle ang in deg about 2800 at max DF and -700 at max PF
    dg_trans = (int32)((float)(-1000/(3*a1*dgaa*dgaa+2*a2*dgaa+a3))); //10x the transmission ratio
}

static void dg_set_ank_torque(int32 torq) //torq is in 100x Nm
{
    int32 strain_gain = 10;
    int32 measured_torq = -(dg_strain_read-32300)*10; //in 100xNm
    
    dg_set_mot_volt((torq-measured_torq));
    
    //dg_set_mot_torque((100*torq)/dg_trans);
}

static void dg_set_mot_torque(int32 torq) //torq is in mNm
{
    
    //motor does not move until 300 mNm are applied
    if (torq>=0)
    {
        
    }
    else
    {
    }
    int32 dg_des_cur = torq*10; //desired motor current in mAmps 
    int32 dg_des_vol = (dg_des_cur*19)/100+as5047.angle_vel_RPM*10; //desired motor voltage in mV
    dg_set_mot_volt(dg_des_vol);

    
}

//Set the motor voltage to mot_volt [mV]
static void dg_set_mot_volt(int32 mot_volt)
{   
    int32 voltsign = 1;
    int32 abvolt = mot_volt;
    if (abvolt<0)
    {
        voltsign = -1;
        abvolt = -abvolt;
    }
    abvolt = abvolt + 400;
    
    //int32 mot_com = voltsign*((abvolt*1024)/(dg_batvolt));
    int32 mot_com = voltsign*abvolt*1024/dg_batvolt*1970/2000;
    
    motor_open_speed_1(mot_com);	
}
