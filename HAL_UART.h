/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// HAL_UART.h - Prototypes of hardware abstraction layer for UART between
//             MSP432P401R and OPT3001
//
//****************************************************************************

#ifndef __HAL_UART_H_
#define __HAL_UART_H_

#define UART1_TXD_PORT        GPIO_PORT_P3
#define UART1_TXD_PIN         GPIO_PIN4

#define UART1_RXD_PORT        GPIO_PORT_P3
#define UART1_RXD_PIN         GPIO_PIN5

#define UART0_TXD_PORT        GPIO_PORT_P4
#define UART0_TXD_PIN         GPIO_PIN2

#define UART0_RXD_PORT        GPIO_PORT_P4
#define UART0_RXD_PIN         GPIO_PIN3

#define USE_BUFFER_UART_TX
//#define USE_BUFFER_UART_RX

#ifdef USE_BUFFER_UART_TX
// USART Transmitter buffer
#define TX0_BUFFER_SIZE 32
#define TX1_BUFFER_SIZE 32
#endif

#ifdef USE_BUFFER_UART_RX
// USART Receiver buffer
#define RX0_BUFFER_SIZE 128
#define RX1_BUFFER_SIZE 128
#endif

#define UART_SELECT_FUNCTION GPIO_PRIMARY_MODULE_FUNCTION

#ifdef USE_BUFFER_UART_TX
extern char tx_buffer0[TX0_BUFFER_SIZE];
    #if TX_BUFFER_SIZE0 <= 256
    extern unsigned char tx_wr_index0,tx_rd_index0,tx_counter0;
    #else
    extern unsigned int tx_wr_index0,tx_rd_index0,tx_counter0;
    #endif
extern char tx_buffer1[TX1_BUFFER_SIZE];
    #if TX1_BUFFER_SIZE <= 256
    extern unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;
    #else
    extern unsigned int tx_wr_index1,tx_rd_index1,tx_counter1;
    #endif
#endif

#ifdef USE_BUFFER_UART_RX
extern char rx_buffer0[RX0_BUFFER_SIZE];
    #if RX_BUFFER_SIZE0 <= 256
    extern unsigned char rx_wr_index0,rx_rd_index0,rx_counter0;
    #else
    extern unsigned int rx_wr_index0,rx_rd_index0,rx_counter0;
    #endif
extern char rx_buffer1[RX1_BUFFER_SIZE];
    #if RX_BUFFER_SIZE1 <= 256
    extern unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;
    #else
    extern unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
    #endif
#endif

    uint8_t (*UART_state)(uint16_t);

void UART_initGPIO(uint16_t baseAddress);
uint8_t UART_init(uint16_t baseAddress, uint32_t   baudRate, uint32_t clockSource);
void UART_puts(uint16_t baseAddress, char *pStr );
//static
//extern inline
void UART_putchar(uint16_t baseAddress, char pChar);
void    set_uart_x(uint16_t baseAddress, bool flag);
#ifdef USE_BUFFER_UART_TX
uint8_t UART_TX_buffer_busy(uint16_t baseAddress);
#endif


#endif /* __HAL_UART_H_ */
