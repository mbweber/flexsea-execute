//****************************************************************************
// MIT Media Lab - Biomechatronics
// Jean-Francois (Jeff) Duval
// jfduval@mit.edu
// 02/2015
//****************************************************************************
// peripherals: code for the general peripheral modules
//****************************************************************************
	
#ifndef INC_PERIPH_H
#define INC_PERIPH_H

//****************************************************************************
// Include(s)
//****************************************************************************	
	
#include "main.h"
	
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_peripherals(void);
void init_tb_timers(void);
void init_angle_timer(void);

void update_counts_since_last_ang_read(void);
void reset_ang_counter(void);
void update_as5047(int32);


//****************************************************************************
// Shared Variable(s):
//****************************************************************************

extern uint8 uart_dma_rx_buf[96];
extern uint8 uart_dma_rx_buf_unwrapped[96];
extern uint8 uart_dma_tx_buf[96];
extern uint8 gui_fsm_flag;

//****************************************************************************
// Definition(s):
//****************************************************************************
	
#endif	//INC_PERIPH_H
	