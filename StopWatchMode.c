/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
/*******************************************************************************
 *
 * StopWatchMode.c
 *
 * Simple stopwatch application that supports counting up and split time.
 *
 * February 2015
 * E. Chen
 *
 ******************************************************************************/

#include <driverlib.h>
#include "StopWatchMode.h"
#include "hal_LCD.h"

void stopWatch()
{
    while(mode == STOPWATCH_MODE)
    {
        // stays in LPM3 while stopwatch is running and wakes up every 10ms to update clock and LCD
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM3
        __no_operation();

        if (stopWatchRunning)
            // Update LCD with new time
            displayTime();
            __no_operation();
    }
}

void stopWatchModeInit()
{
    stopWatchRunning = 0;
    displayScrollText("STOPWATCH MODE",LCD_MEMORY_MAIN);

    LCD_C_selectDisplayMemory(LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY);

    // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
    RTC_C_holdClock(RTC_C_BASE);

    // Clear stopwatch
    resetStopWatch();

    RTC_C_definePrescaleEvent(RTC_C_BASE, RTC_C_PRESCALE_0, RTC_C_PSEVENTDIVIDER_32);
    RTC_C_enableInterrupt(RTC_C_BASE, RTC_C_PRESCALE_TIMER0_INTERRUPT | RTC_C_CLOCK_READ_READY_INTERRUPT | RTC_C_TIME_EVENT_INTERRUPT);
    RTC_C_startCounterPrescale(RTC_C_BASE, RTC_C_PRESCALE_0);

    S1buttonDebounce = 0;
    S2buttonDebounce = 0;

    // Check if any button is pressed
    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
    __bis_SR_register(LPM3_bits | GIE);         // enter LPM3
    __no_operation();

    // Disable ADC12, TimerA1, Internal Ref and Temp used by TempSensor Mode
    ADC12_B_disable(ADC12_B_BASE);
    ADC12_B_disableConversions(ADC12_B_BASE,true);

    Timer_A_stop(TIMER_A1_BASE);

//    __bis_SR_register(LPM3_bits | GIE);         // enter LPM3
}

void resetStopWatch()
{
    //Setup Current Time for Calendar
    currentTime.Seconds = 0x00;
    currentTime.Minutes = 0x00;
    currentTime.Hours = 0x00;
    currentTime.DayOfWeek = 0x04;
    currentTime.DayOfMonth = 0x30;
    currentTime.Month = 0x04;
    currentTime.Year = 0x2015;
    centisecond = 0;

    RTC_C_initCounter(RTC_C_BASE, RTC_C_CLOCKSELECT_32KHZ_OSC, RTC_C_COUNTERSIZE_16BIT);
    RTC_C_initCalendar(RTC_C_BASE,
                       &currentTime,
                       RTC_C_FORMAT_BINARY);

    // Update LCD with new time
    displayTime();
}

void displayTime()
{
    currentTime = RTC_C_getCalendarTime(RTC_C_BASE);
    // Display Minute, Second, Centiseconds if below 1 hour mark.
    if ((int)(currentTime.Hours) == 0)
    {
        showChar((centisecond/327) % 10 + '0',pos6,LCD_MEMORY_MAIN);
        showChar((centisecond/327) / 10 + '0',pos5,LCD_MEMORY_MAIN);
        showChar((currentTime.Seconds) % 10 + '0',pos4,LCD_MEMORY_MAIN);
        showChar((currentTime.Seconds) / 10 + '0',pos3,LCD_MEMORY_MAIN);
        showChar((currentTime.Minutes) % 10 + '0',pos2,LCD_MEMORY_MAIN);
        showChar((currentTime.Minutes) / 10 + '0',pos1,LCD_MEMORY_MAIN);
    }
    // Otherwise, display Hour, Minute, Second
    else
    {
        // Stop counter for centiseconds
        RTC_C_holdCounterPrescale(RTC_C_BASE, RTC_C_PRESCALE_0);

        showChar((currentTime.Seconds) % 10 + '0',pos6,LCD_MEMORY_MAIN);
        showChar((currentTime.Seconds) / 10 + '0',pos5,LCD_MEMORY_MAIN);
        showChar((currentTime.Minutes) % 10 + '0',pos4,LCD_MEMORY_MAIN);
        showChar((currentTime.Minutes) / 10 + '0',pos3,LCD_MEMORY_MAIN);
        showChar((currentTime.Hours) % 10 + '0',pos2,LCD_MEMORY_MAIN);
        showChar((currentTime.Hours) / 10 + '0',pos1,LCD_MEMORY_MAIN);
    }

    // Blink Stopwatch symbol
    if (centisecond/327 == 0)
    {
        LCDM3 |= 0x08;
        LCDBM3 |= 0x08;
    }
    if (centisecond/327 == 50)
    {
        LCDM3 &= ~0x08;
        LCDBM3 &= ~0x08;
    }

    // Display the 2 colons
    LCDM7 |= 0x04;
    LCDBM7 |= 0x04;
    LCDM20 |= 0x04;
    LCDBM20 |= 0x04;
}
