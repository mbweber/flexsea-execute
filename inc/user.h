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
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user: User Projects & Functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

/*Important: we reached a point where we couldn't support all configurations
  without changing the TopDesign (we ran out of ressources). You might have
  to select a different TopDesign file than the one included by default (check
  the folded, there is more than one included) */

#ifndef INC_USER_H
#define INC_USER_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
#include "user_exo.h"
#include "user_csea_knee.h"	
#include "user_ricnu_knee.h"	
#include "user_ankle_2dof.h"
#include "user_dsdm.h"
#include "user_gravity_1.h"
//Add your project specific user_x.h file here
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_user(void);
void user_fsm(void);

//****************************************************************************
// Definition(s):
//****************************************************************************	

//List of encoders that can be used by controllers (position, impedance),
//and for commutation:
#define ENC_NONE				0	//No encoder
#define ENC_HALL				1	//Hall effect (motor commutation)
#define ENC_QUADRATURE			2	//Optical or magnetic, AB/I inputs on QEI1
#define ENC_ANALOG				3	//Potentiometer (or other), on ext. analog in.
#define ENC_AS5047				4	//16-bit Magnetic Position Sensor, SPI
#define ENC_AS5048B				5	//14-bit Magnetic Position Sensor, I2C
#define ENC_CUSTOM              6   //Heavily modified user variable that cannot 
									//be represented CTRL_ENC_FCT
//(later you'll assign what encoder is used by the controllers, for motor
// commutation, and which one is displayed in the GUI)

//Type of motor commutation:
#define COMMUT_BLOCK			0
#define COMMUT_SINE				1

//List of projects:
#define PROJECT_BAREBONE		0	//Barebone Execute, default option.
#define PROJECT_EXOCUTE			1	//ExoBoot, everything running on Execute
#define PROJECT_CSEA_KNEE		2	//CSEA Knee + FlexSEA
#define PROJECT_RICNU_KNEE		3	//RIC/NU Knee
#define PROJECT_ANKLE_2DOF		4	//Biomechatronics 2-DOF Ankle
#define PROJECT_DSDM			5	//MIT d'Arbeloff Dual-Speed Dual-Motor
#define PROJECT_DEPHY_GRAVITY_1	6	//Dephy's Gravity Boot v1

//List of sub-projects:
#define SUBPROJECT_NONE			0
#define SUBPROJECT_A			1
#define SUBPROJECT_B			2
//(ex.: the 2-DoF ankle has 2 Execute. They both use PROJECT_2DOF_ANKLE, and each
// 		of them has a sub-project for specific configs)

//Step 1) Select active project (from list):
//==========================================

#define ACTIVE_PROJECT			PROJECT_DEPHY_GRAVITY_1
#define ACTIVE_SUBPROJECT		SUBPROJECT_A

//Step 2) Customize the enabled/disabled sub-modules:
//===================================================

//Barebone FlexSEA-Execute project - no external peripherals.
#if(ACTIVE_PROJECT == PROJECT_BAREBONE)
	
	//Enable/Disable sub-modules:
	#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	#define USE_QEI
	#define USE_TRAPEZ
	#define USE_I2C_0			//3V3, IMU & Expansion.
	#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	#define USE_IMU				//Requires USE_I2C_0
	#define USE_STRAIN			//Requires USE_I2C_1
	
	//Motor type, direction and commutation:
	#define MOTOR_TYPE		MOTOR_BRUSHLESS
	#define PWM_SIGN		1
	#define MOTOR_COMMUT 	COMMUT_BLOCK
	#define CURRENT_ZERO	((int32)2125)
	
	//Runtime finite state machine (FSM):
	#define RUNTIME_FSM		DISABLED
	
	//Encoders:
	#define ENC_CONTROL		ENC_QUADRATURE
	#define ENC_COMMUT		ENC_HALL
	#define ENC_DISPLAY		ENC_QUADRATURE	
	
	//Control encoder function:
	#define CTRL_ENC_FCT(x) (x)	
	
	//Slave ID:
	#define SLAVE_ID		FLEXSEA_EXECUTE_1
	
	//Project specific definitions:
	//...
	
#endif	//PROJECT_BAREBONE

//Exoboot
#if(ACTIVE_PROJECT == PROJECT_EXOCUTE)
	
	//Enable/Disable sub-modules:
	#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	#define USE_QEI
	#define USE_TRAPEZ
	#define USE_I2C_0			//3V3, IMU & Expansion.
	#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	#define USE_IMU				//Requires USE_I2C_0
	#define USE_STRAIN			//Requires USE_I2C_1
	
	//Motor type, direction and commutation:
	#define MOTOR_COMMUT 	COMMUT_BLOCK
	#define MOTOR_TYPE		MOTOR_BRUSHLESS
	#define PWM_SIGN		1
	
	//Runtime finite state machine (FSM):
	#define RUNTIME_FSM		DISABLED

	//Encoders:
	#define ENC_CONTROL		ENC_QUADRATURE
	#define ENC_COMMUT		ENC_HALL
	#define ENC_DISPLAY		ENC_CONTROL
	
	//Control encoder function:
	#define CTRL_ENC_FCT(x) (x)	
	
	//Project specific definitions:
	//...
	
#endif	//PROJECT_EXOCUTE

//CSEA Knee
#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
	
	//Enable/Disable sub-modules:
	#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	#define USE_QEI
	#define USE_TRAPEZ
	#define USE_I2C_0			//3V3, IMU & Expansion.
	#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	#define USE_IMU				//Requires USE_I2C_0
	//#define USE_STRAIN		//Requires USE_I2C_1
	
	//Motor type, direction and commutation:
	#define MOTOR_COMMUT 	COMMUT_BLOCK
	#define MOTOR_TYPE		MOTOR_BRUSHLESS
	#define PWM_SIGN		1
	
	//Runtime finite state machine (FSM):
	#define RUNTIME_FSM		ENABLED

	//Encoders:
	#define ENC_CONTROL		ENC_ANALOG
	#define ENC_COMMUT		ENC_HALL
	#define ENC_DISPLAY		ENC_CONTROL	
	
	//Project specific definitions:
	#define CSEA_FULL_FLEX_RAW		580		//Raw value
	#define CSEA_FULL_EXT_RAW		2840	//Raw value
	#define CSEA_FULL_FLEX			2260
	#define CSEA_FULL_EXT			0
	#define CSEA_MARGIN				300
	#define CSEA_MOTION_TIME		3500
	
	//Control encoder function:
	#define CTRL_ENC_FCT(x) (-((int16)x - CSEA_FULL_EXT_RAW))
	
#endif	//PROJECT_CSEA_KNEE

//RIC/NU Knee
#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
	
	//Enable/Disable sub-modules:
	#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	//#define USE_QEI
	#define USE_TRAPEZ
	#define USE_I2C_0			//3V3, IMU & Expansion.
	#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	#define USE_IMU				//Requires USE_I2C_0
	//#define USE_STRAIN		//Requires USE_I2C_1
	#define USE_AS5047			//16-bit Position Sensor, SPI
	//#define USE_MINM_RGB		//External RGB LED. Requires USE_I2C_0.
	#define USE_EEPROM			//Non-volatile memory, EEPROM
	//#define USE_FLASH			//Non-volatile memory, FLASH
	//#define USE_BLUETOOTH		//Bluetooth module on EX12/EX13
//	#define USE_EXT_I2C_STRAIN	//External Strain Amplifier, on I2C0
	#define USE_AS5048B			//14-bit Position Sensor, on I2C0
	
	//Motor type, direction and commutation:
	#define MOTOR_COMMUT 			COMMUT_SINE
	#define MOTOR_TYPE				MOTOR_BRUSHLESS
	#define PWM_SIGN				1
	
	//Define if you want to find the poles:
    //#define FINDPOLES 		
     
    //Runtime finite state machine (FSM):         
    #ifdef FINDPOLES
        #define RUNTIME_FSM     	DISABLED
    #else
        #ifdef USE_TRAPEZ
            #define RUNTIME_FSM     DISABLED
        #else
            #define RUNTIME_FSM     ENABLED
        #endif
    #endif

	//Encoders:
	#define ENC_CONTROL				ENC_AS5048B
	#define ENC_COMMUT				ENC_AS5047
	#define ENC_DISPLAY				ENC_CONTROL	
	
	//Control encoder function:
	#define CTRL_ENC_FCT(x) 		(14000 - x)	//ToDo make better
	
	//Project specific definitions:
	#define CURRENT_ZERO			((int32)2127)
	
#endif	//PROJECT_RICNU_KNEE

//MIT 2-DoF Ankle
#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
     
    //Enable/Disable sub-modules:
    #define USE_RS485
    #define USE_USB
    #define USE_COMM            //Requires USE_RS485 and/or USE_USB
    //#define USE_QEI
    #define USE_TRAPEZ
    #define USE_I2C_0           //3V3, IMU & Expansion.
    #define USE_I2C_1           //5V, Safety-CoP & strain gauge pot.
    #define USE_IMU             //Requires USE_I2C_0
    //#define USE_STRAIN        //Requires USE_I2C_1
    #define USE_AS5047          //16-bit Position Sensor, SPI
    #define USE_SPI_COMMUT      //
     
    //Motor type and commutation:
	#define MOTOR_COMMUT 	COMMUT_BLOCK
    #define MOTOR_TYPE      MOTOR_BRUSHLESS
     
    //Runtime finite state machine (FSM):
     
    //#define FINDPOLES //define if you want to find the poles
     
    #ifdef FINDPOLES
        #define RUNTIME_FSM     DISABLED
    #else
        #ifdef USE_TRAPEZ
            #define RUNTIME_FSM     DISABLED
        #else
            #define RUNTIME_FSM     ENABLED
        #endif
    #endif
     
     
             
    //Encoders:
    #define ENC_CONTROL     ENC_CUSTOM
    #define ENC_COMMUT      ENC_AS5047
    #define ENC_DISPLAY     ENC_CONTROL 
     
     
     
    //Subproject A: Left execute board looking at the back of the ankle while it is standing up
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
         
        //Control encoder function:
 
        #define PWM_SIGN          1
        #define CURRENT_ZERO            ((int32)2060)   
        #define CTRL_ENC_FCT(x) (x) 
        //...
		
		//Slave ID:
		#define SLAVE_ID		FLEXSEA_EXECUTE_2
         
    #endif  //SUBPROJECT_A
     
    //Subproject B: Right actuator
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
         
        //Control encoder function:
        #define PWM_SIGN         -1
        #define CURRENT_ZERO            ((int32)2125)
        #define CTRL_ENC_FCT(x) (x) 
         
        //...
		
		//Slave ID:
	#define SLAVE_ID		FLEXSEA_EXECUTE_1
         
    #endif  //SUBPROJECT_A
     
    //Project specific definitions:
    extern int32 ankle_ang, ankle_trans, mot_vel;
    //...   
     
#endif  //PROJECT_ANKLE_2DOF

//MIT d'Arbeloff Dual-Speed Dual-Motor
#if(ACTIVE_PROJECT == PROJECT_DSDM)
	
	//Enable/Disable sub-modules:
	#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	#define USE_QEI
	#define USE_TRAPEZ
	#define USE_I2C_0			//3V3, IMU & Expansion.
	#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	#define USE_IMU				//Requires USE_I2C_0
	//#define USE_STRAIN			//Requires USE_I2C_1
	
	//Motor type, direction and commutation:
	#define MOTOR_COMMUT 	COMMUT_BLOCK
	#define MOTOR_TYPE		MOTOR_BRUSHED
	
	//Runtime finite state machine (FSM):
	#define RUNTIME_FSM		ENABLED

	//Encoders:
	#define ENC_CONTROL		ENC_QUADRATURE
	#define ENC_COMMUT		ENC_NONE		//Brushed, no encoder
	#define ENC_DISPLAY		ENC_CONTROL
	
	//Subproject A: Fast actuator
	#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
		
		//Control encoder function:
		#define CTRL_ENC_FCT(x) (x)	//ToDo
		#define PWM_SIGN		1
		
		//...
		
	#endif	//SUBPROJECT_A
	
	//Subproject B: Slow actuator
	#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
		
		//Control encoder function:
		#define CTRL_ENC_FCT(x) (x)	//ToDo
		#define PWM_SIGN		1
		
		//...
		
	#endif	//SUBPROJECT_B	
	
	//Project specific definitions:
	//...
	
#endif	//PROJECT_DSDM

//Dephy's Gravity Boot v1
#if(ACTIVE_PROJECT == PROJECT_DEPHY_GRAVITY_1)
     
    //Enable/Disable sub-modules:
    #define USE_RS485
    #define USE_USB
    #define USE_COMM            //Requires USE_RS485 and/or USE_USB
    //#define USE_QEI
    #define USE_TRAPEZ
    #define USE_I2C_0           //3V3, IMU & Expansion.
    #define USE_I2C_1           //5V, Safety-CoP & strain gauge pot.
    #define USE_IMU             //Requires USE_I2C_0
    #define USE_STRAIN        //Requires USE_I2C_1
    #define USE_AS5047          //16-bit Position Sensor, SPI
    //#define USE_SPI_COMMUT      //
	#define USE_EEPROM			//Non-volatile memory, EEPROM
	//#define USE_FLASH			//Non-volatile memory, FLASH
	//#define USE_BLUETOOTH		//Bluetooth module on EX12/EX13
     
    //Motor type and commutation:
	#define MOTOR_COMMUT 	COMMUT_SINE
    #define MOTOR_TYPE      MOTOR_BRUSHLESS
	
    //#define FINDPOLES 		//define if you want to find the poles
     
    //Runtime finite state machine (FSM):    
     
    #ifdef FINDPOLES
        #define RUNTIME_FSM     	DISABLED
    #else
        #ifdef USE_TRAPEZ
            #define RUNTIME_FSM     DISABLED
        #else
            #define RUNTIME_FSM     ENABLED
        #endif
    #endif     
             
    //Encoders:
    #define ENC_CONTROL     ENC_AS5047		//ENC_CUSTOM
    #define ENC_COMMUT      ENC_AS5047
    #define ENC_DISPLAY     ENC_CONTROL      
     
    //Subproject A: Left foot
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
         
        //Control encoder function:
 
        #define PWM_SIGN			(-1) //1 for exo, -1 for test bench
        #define CURRENT_ZERO		((int32)2127) //1750 for test bench   
        #define CTRL_ENC_FCT(x) 	(x) 
        //...
		
		//Slave ID:
		#define SLAVE_ID			FLEXSEA_EXECUTE_1
         
    #endif  //SUBPROJECT_A
     
    //Subproject B: Right foot
    #if(ACTIVE_SUBPROJECT == SUBPROJECT_B)
         
        //Control encoder function:
        #define PWM_SIGN         	-1
        #define CURRENT_ZERO		((int32)2125)
        #define CTRL_ENC_FCT(x)		(x) 
         
        //...
		
		//Slave ID:
	#define SLAVE_ID				FLEXSEA_EXECUTE_2
         
    #endif  //SUBPROJECT_A
     
    //Project specific definitions & variables:
    extern int32 ankle_ang, ankle_trans, mot_vel;
    //...   
     
#endif  //PROJECT_DEPHY_GRAVITY_1

//****************************************************************************
// Structure(s)
//****************************************************************************	
	
#endif	//INC_USER_H
