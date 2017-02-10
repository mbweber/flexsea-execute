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
	[Contributors]
*****************************************************************************
	[This file] isr_callbacks: Implementation of the ISR functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef CYAPICALLBACKS_H
#define CYAPICALLBACKS_H

	//Define callbacks here to enable them:
	#define isr_t1_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_t2_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_sar1_dma_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_sar2_dma_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_dma_uart_rx_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_dma_uart_tx_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_dma_uart_bt_rx_INTERRUPT_INTERRUPT_CALLBACK
	#define isr_delsig_INTERRUPT_INTERRUPT_CALLBACK
	#define ADC_SAR_1_ISR_INTERRUPT_CALLBACK
	#define isr_spi_tx_INTERRUPT_INTERRUPT_CALLBACK
    
	//And include their prototype:
	void isr_t1_Interrupt_InterruptCallback();
	void isr_t2_Interrupt_InterruptCallback();
	void isr_sar1_dma_Interrupt_InterruptCallback();
	void isr_sar2_dma_Interrupt_InterruptCallback();
	void isr_dma_uart_rx_Interrupt_InterruptCallback();
	void isr_dma_uart_tx_Interrupt_InterruptCallback();
	void isr_dma_uart_bt_rx_Interrupt_InterruptCallback();
	void isr_delsig_Interrupt_InterruptCallback();
	void ADC_SAR_1_ISR_InterruptCallback();
	void isr_spi_tx_Interrupt_InterruptCallback();
	//Place all the functions in isr_callback.c
    
#endif //CYAPICALLBACKS_H  
