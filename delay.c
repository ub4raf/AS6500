/*
 * delay.c
 *
 *  Created on: Jan 7, 2020
 *      Author: Sanchez
 */
#include <driverlib.h>
#include "delay.h"

#ifdef USE_DUMMY_DELAY
void delay_s(uint16_t    s)
{
    while (s--)
        __delay_cycles(SMCLK_F);
}

void delay_ms(uint16_t    ms)
{
    while (ms--)
        //__delay_cycles(16000); // set for 16Mhz change it to 1000 for 1 Mhz
        // __delay_cycles(8000); // set for 8Mhz change it to 1000 for 1 Mhz
        //__delay_cycles(SMCLK_F/1000);
        delay_us(1000);
}


void delay_us(uint16_t us)
{
    while (us--)
        // __delay_cycles(8); // set for 16Mhz change it to 1000 for 1 Mhz
        // __delay_cycles(4); // set for 8Mhz change it to 1000 for 1 Mhz
        // __delay_cycles(2);//
        __delay_cycles(1);
}
#else
void delay_s(uint16_t    s)
{
  if(s)
{       //Start timer in continuous mode sourced by ACLK
       Timer_A_initContinuousModeParam initContParam = {0};
       initContParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
       initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
       initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
       initContParam.timerClear = TIMER_A_DO_CLEAR;
       initContParam.startTimer = false;

       Timer_A_initCompareModeParam initCompParam = {0};
       initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
       initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
       initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
       initCompParam.compareValue = 0x7FFF;

       while(s--)
    {
       Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);
       //Initiaze compare mode
       Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
           TIMER_A_CAPTURECOMPARE_REGISTER_0
           );
       Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);
       Timer_A_startCounter( TIMER_A1_BASE,
               TIMER_A_CONTINUOUS_MODE
                   );
        //Enter LPM3, enable interrupts
        __bis_SR_register(LPM0_bits + GIE);
    }
  Timer_A_stop(TIMER_A1_BASE);
}
}

void delay_ms(uint16_t    ms)
{
    if(ms)
  {
        if(ms>23) ms=ms-ms/11;  // dirty hack
        //TODO  variable capture
        //uint32_t    clicks;
        //uint16_t    iterations;
        //clicks=ms*32768/1000;
        //iterations=clicks/0xFFFF;
         //Start timer in continuous mode sourced by ACLK
         Timer_A_initContinuousModeParam initContParam = {0};
         initContParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
         initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
         initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
         initContParam.timerClear = TIMER_A_DO_CLEAR;
         initContParam.startTimer = false;

         Timer_A_initCompareModeParam initCompParam = {0};
         initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
         initCompParam.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
         initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
         initCompParam.compareValue = 33;//clicks%0xFFFF;

         while(ms--)//iterations--)
      {
         Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);
         //Initiaze compare mode
         Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
             TIMER_A_CAPTURECOMPARE_REGISTER_0
             );
         Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);
         Timer_A_startCounter( TIMER_A1_BASE,
                 TIMER_A_CONTINUOUS_MODE
                     );
          //Enter LPM3, enable interrupts
          __bis_SR_register(LPM0_bits + GIE);
      }
  Timer_A_stop(TIMER_A1_BASE);
  }
}

void delay_us(uint16_t us)
{
    while (us--)
        // __delay_cycles(8); // set for 16Mhz
        // __delay_cycles(4); // set for 8Mhz
        // __delay_cycles(2);//
        __delay_cycles(1);
}

//таймерная задержка delay_ms(); Только пробуждение.
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR (void)
{
    Timer_A_getCaptureCompareCount(TIMER_A1_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0);
    __bic_SR_register_on_exit(LPM0_bits);
}

#endif

