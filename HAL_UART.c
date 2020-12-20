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
// HAL_UART.c - Hardware abstraction layer for UART with MSP432P401R
//
//****************************************************************************

#include <driverlib.h>
#include "HAL_UART.h"

#ifdef USE_BUFFER_UART_TX
char tx_buffer0[TX0_BUFFER_SIZE]={0};
    #if TX_BUFFER_SIZE0 <= 256
    uint8_t tx_wr_index0=0,tx_rd_index0=0,tx_counter0=0;
    #else
    uint16_t tx_wr_index0=0,tx_rd_index0=0,tx_counter0=0;
    #endif
char tx_buffer1[TX1_BUFFER_SIZE]={0};
    #if TX1_BUFFER_SIZE <= 256
    uint8_t tx_wr_index1=0,tx_rd_index1=0,tx_counter1=0;
    #else
    uint16_t tx_wr_index1=0,tx_rd_index1=0,tx_counter1=0;
    #endif
#endif

#ifdef USE_BUFFER_UART_RX
char rx_buffer1[RX1_BUFFER_SIZE]={0};
    #if RX_BUFFER_SIZE1 <= 256
    uint8_t rx_wr_index1,rx_rd_index1,rx_counter1;
    #else
    uint16_t rx_wr_index1,rx_rd_index1,rx_counter1;
    #endif
char rx_buffer0[RX0_BUFFER_SIZE]={0};
    #if RX_BUFFER_SIZE0 <= 256
    uint8_t rx_wr_index0,rx_rd_index0,rx_counter0;
    #else
    uint16_t rx_wr_index0,rx_rd_index0,rx_counter0;
    #endif
#endif

    uint8_t (*UART_state)(uint16_t);

#ifdef USE_BUFFER_UART_TX
   //наличие байтов в буфере на передачу:
uint8_t UART_TX_buffer_busy(uint16_t baseAddress)
    {
        switch(baseAddress)
        {
            case    EUSCI_A1_BASE:
                return  tx_counter1;
            case    EUSCI_A0_BASE:
                return  tx_counter0;
        }
        return 0;
    }
#endif

/* Initializes Backchannel UART GPIO */
void UART_initGPIO(uint16_t baseAddress)
{
    switch(baseAddress)
        {

            case    EUSCI_A1_BASE:
                GPIO_setAsPeripheralModuleFunctionInputPin(
                        UART1_TXD_PORT,
                        UART1_TXD_PIN,
                        UART_SELECT_FUNCTION);

                GPIO_setAsPeripheralModuleFunctionInputPin(
                        UART1_RXD_PORT,
                        UART1_RXD_PIN,
                        UART_SELECT_FUNCTION);
            break;
            case    EUSCI_A0_BASE:
                GPIO_setAsPeripheralModuleFunctionInputPin(
                        UART0_TXD_PORT,
                        UART0_TXD_PIN,
                        UART_SELECT_FUNCTION);

                GPIO_setAsPeripheralModuleFunctionInputPin(
                        UART0_RXD_PORT,
                        UART0_RXD_PIN,
                        UART_SELECT_FUNCTION);
            break;
        }

}

uint8_t    UART_disable(uint16_t baseAddress)
    {
        if((baseAddress!=EUSCI_A0_BASE)&&(baseAddress!=EUSCI_A1_BASE))
                return 2;
        EUSCI_A_UART_disableInterrupt(baseAddress,
                             EUSCI_A_UART_RECEIVE_INTERRUPT
                             |EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                             |EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT );
        EUSCI_A_UART_clearInterrupt(baseAddress,
                              EUSCI_A_UART_RECEIVE_INTERRUPT
                             |EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                             |EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT);
        EUSCI_A_UART_disable(baseAddress);
        // TODO disable rx pin??
        return 0;
    }

uint8_t UART_init(uint16_t baseAddress, uint32_t   baudRate, uint32_t clockSource)
{

    /* UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 115200 baud rate. These
     * values were calculated using the online calculator that TI provides
     * at:
     *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    if((baseAddress!=EUSCI_A0_BASE)&&(baseAddress!=EUSCI_A1_BASE))
            return 2;
    EUSCI_A_UART_initParam param = {0};
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;

    if(clockSource==32768)
    {
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_ACLK;
        switch(baudRate)
            {
        //case:
            case    1200:
                param.clockPrescalar = 1;                                             // UCBRx
                param.firstModReg = 11;                                                  // UCBRFx
                param.secondModReg = 37;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    2400:
                param.clockPrescalar = 13;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 182;
                param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    4800:
                param.clockPrescalar = 6;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 238;
                param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    9600:
                param.clockPrescalar = 3;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 146;
                param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0
            break;
            case    14400:
                param.clockPrescalar = 2;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 68;
                param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

            break;
            case    19200:
                param.clockPrescalar = 1;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 183;
                param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

            break;
            default:
                return 1;
            }

    }
    else
    {
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        switch(clockSource/baudRate)
            {
        case    4:
        case    5:   //1M 115200
            param.clockPrescalar = 8;                                             // UCBRx
            param.firstModReg = 0;                                                  // UCBRFx
            param.secondModReg = 214;
            param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;  // Oversampling
        break;
        case    9:
        case    8:   //1M 115200
            param.clockPrescalar = 8;                                             // UCBRx
            param.firstModReg = 0;                                                  // UCBRFx
            param.secondModReg = 214;
            param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;  // Oversampling
        break;
        case    17:
        case    18:   //2M 115200
            param.clockPrescalar = 17;                                             // UCBRx
            param.firstModReg = 0;                                                  // UCBRFx
            param.secondModReg = 74;
            param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;  // Oversampling
        break;
            case    35:
            case    34:   //4M 115200
                param.clockPrescalar = 2;                                             // UCBRx
                param.firstModReg = 2;                                                  // UCBRFx
                param.secondModReg = 187;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
        //case:
            case    69:   //8M 115200
                param.clockPrescalar = 4;                                             // UCBRx
                param.firstModReg = 5;                                                  // UCBRFx
                param.secondModReg = 85;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    833:    //8M 9600
                param.clockPrescalar = 52;                                             // UCBRx
                param.firstModReg = 1;                                                  // UCBRFx
                param.secondModReg = 73;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    6666:  //8M 1200
            case    6667:
                param.clockPrescalar = 416;                                             // UCBRx
                param.firstModReg = 10;                                                  // UCBRFx
                param.secondModReg = 182;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    555:  //8M 14400
            case    556:
                param.clockPrescalar = 34;                                             // UCBRx
                param.firstModReg = 11;                                                  // UCBRFx
                param.secondModReg = 170;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    416:  //8M 19200
            case    417:
                param.clockPrescalar = 26;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 214;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    277:  //8M 28800
            case    278:
                param.clockPrescalar = 17;                                             // UCBRx
                param.firstModReg = 5;                                                  // UCBRFx
                param.secondModReg = 221;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    208:  //8M 38400
            case    209:
                param.clockPrescalar = 13;                                             // UCBRx
                param.firstModReg = 0;                                                  // UCBRFx
                param.secondModReg = 69;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            case    138:  //8M 57600
            case    139:
                param.clockPrescalar = 8;                                             // UCBRx
                param.firstModReg = 10;                                                  // UCBRFx
                param.secondModReg = 247;
                param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;  // Oversampling
            break;
            default:
                return 1;
            }
    }
    UART_initGPIO(baseAddress);

    /* Configuring UART Module */
    if(STATUS_FAIL == EUSCI_A_UART_init(baseAddress, &param))
        {
            //while(1)
            return 3;
                //displayScrollText("ERROR UART");
        }

    /* Enable UART module */
    EUSCI_A_UART_enable(baseAddress);
#ifdef USE_BUFFER_UART_TX
    if(baseAddress==EUSCI_A0_BASE)
        {
                tx_wr_index0=0;
                tx_rd_index0=0;
                tx_counter0=0;
        }
    else
        {
                tx_wr_index1=0;
                tx_rd_index1=0;
                tx_counter1=0;
        }
#endif

    EUSCI_A_UART_clearInterrupt(baseAddress,
                         EUSCI_A_UART_RECEIVE_INTERRUPT
#ifdef USE_BUFFER_UART_TX
                         //|EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                         //|EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT
#endif
                         );

    // Enable USCI RX interrupt
    EUSCI_A_UART_enableInterrupt(baseAddress,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT
#ifdef USE_BUFFER_UART_TX
                                 //|EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                                 //|EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT
#endif
                                 );

	return 0;
}

void    set_uart_x(uint16_t baseAddress, bool flag)
{
        if(flag)
            {
                UART_init(baseAddress,9600,CS_getACLK());
                UART_puts(EUSCI_A1_BASE,"U-ON ");
            }
        else
            {
                UART_disable(baseAddress);
                UART_puts(EUSCI_A1_BASE,"U-OFF ");
            }
}

/* Transmits String over UART */
void UART_puts(uint16_t baseAddress, char *pStr )
{
    while( *pStr )
    {
        UART_putchar(baseAddress, *pStr );
        pStr++;
    }
}

#ifdef USE_BUFFER_UART_TX

void UART_putchar(uint16_t baseAddress, char pChar )
{
    if(baseAddress==EUSCI_A0_BASE)  //speed most valuable then size
    {
        if(UART_state(EUSCI_A1_BASE))
    while (tx_counter0 == TX0_BUFFER_SIZE);
        //__disable_interrupt();
           tx_buffer0[tx_wr_index0++]=pChar;
            #if TX0_BUFFER_SIZE != 256
               if (tx_wr_index0 == TX0_BUFFER_SIZE) tx_wr_index0=0;
            #endif
               ++tx_counter0;
               EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG );
        //__enable_interrupt();
    }
    else
    if(baseAddress==EUSCI_A1_BASE)
        {
            if(UART_state(EUSCI_A1_BASE))
        while (tx_counter1 == TX1_BUFFER_SIZE);
        //__disable_interrupt();
           tx_buffer1[tx_wr_index1++]=pChar;
        #if TX1_BUFFER_SIZE != 256
           if (tx_wr_index1 == TX1_BUFFER_SIZE) tx_wr_index1=0;
        #endif
           ++tx_counter1;
           EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG );
        //__enable_interrupt();
        }
}

/*
*/
//******************************************************************************
//
//This is the USCI_A1 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void EUSCI_A1_ISR(void)
{
    char RXData;
  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
       RXData = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
       //showChar(RXData,pos1, LCD_MEMORY_MAIN);
       //UART_putchar(EUSCI_A0_BASE,RXData);
       switch(RXData)
           {

       case 'r':
           WDT_A_hold(0);
           break;
       case 'a':
           break;
       case 's':
           break;
       case 'd':
           break;
       case ' ':
           break;
       case 'q':
       break;
       case ',':
       break;

       case 'e':
           __bic_SR_register_on_exit(LPM3_bits);
       break;
       case 'z':
          __bic_SR_register_on_exit(LPM3_bits);
      break;
       case 'x':
             __bic_SR_register_on_exit(LPM3_bits);
         break;

       case 'p':
       break;
       case 'P':
           __bic_SR_register_on_exit(LPM3_bits);
       break;
       case 'C':
       break;
       case 'S':
           __bic_SR_register_on_exit(LPM3_bits);
       break;
       case 'T':
          __bic_SR_register_on_exit(LPM3_bits);
      break;
       case 'L':
           __bic_SR_register_on_exit(LPM3_bits);
       break;
       case '0':
           __bic_SR_register_on_exit(LPM3_bits);
       break;
       case '1':
          __bic_SR_register_on_exit(LPM3_bits);
      break;
       case '2':
             __bic_SR_register_on_exit(LPM3_bits);
         break;
       case '3':
             __bic_SR_register_on_exit(LPM3_bits);
         break;
       default:
           //showChar(RXData,0, LCD_MEMORY_MAIN);
       break;
           }
      break;
    case USCI_UART_UCTXIFG:
#ifdef USE_BUFFER_UART_TX       // отправили байт.
        if (tx_counter1)        // Сдвигаем кольцевой буфер назад, если он не пуст
           {
           --tx_counter1;
           EUSCI_A_UART_transmitData(EUSCI_A1_BASE, tx_buffer1[tx_rd_index1++]);
        #if TX1_BUFFER_SIZE != 256
           if (tx_rd_index1 == TX1_BUFFER_SIZE) tx_rd_index1=0;
        #endif
           }
        else                    //  иначе выключаем прерывание.
            {
            EUSCI_A_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
            HWREG16(EUSCI_A1_BASE + OFS_UCAxIFG) |= EUSCI_A_UART_TRANSMIT_INTERRUPT;
            }
#endif
        break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG:
        break;
  }
}
            //  полностью аналогичен предыдущему
//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void EUSCI_A0_ISR(void)
{
    //char RXData;
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
       //RXData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
       //showChar(RXData,pos6, LCD_MEMORY_MAIN);
       /// RX callback or buffer
      break;
    case USCI_UART_UCTXIFG:
#ifdef USE_BUFFER_UART_TX
        if (tx_counter0)
           {
           --tx_counter0;
           EUSCI_A_UART_transmitData(EUSCI_A0_BASE, tx_buffer0[tx_rd_index0++]);
        #if TX0_BUFFER_SIZE != 256
           if (tx_rd_index0 == TX0_BUFFER_SIZE) tx_rd_index0=0;
        #endif
           }
        else
            {
            EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
            HWREG16(EUSCI_A0_BASE + OFS_UCAxIFG) |= EUSCI_A_UART_TRANSMIT_INTERRUPT;
            }
#endif
        break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG:
        break;
  }
}


#else
/* Transmits String over UART */
inline void UART_putchar(uint16_t baseAddress, char pChar )
{
    EUSCI_A_UART_transmitData(baseAddress, pChar );
}
#endif

