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
 * StopWatchMode.h
 *
 * Simple stopwatch application that supports counting up and split time.
 *
 * February 2015
 * E. Chen
 *
 ******************************************************************************/

#ifndef OUTOFBOX_MSP430FR6989_STOPWATCHMODE_H_
#define OUTOFBOX_MSP430FR6989_STOPWATCHMODE_H_

#define STOPWATCH_MODE        1

extern volatile unsigned char mode;
extern volatile unsigned char S1buttonDebounce;
extern volatile unsigned char S2buttonDebounce;
extern Timer_A_initUpModeParam initUpParam_A0;
extern volatile unsigned char stopWatchRunning;
extern Calendar currentTime;
extern volatile unsigned int counter;
extern volatile int centisecond;

void stopWatch(void);
void stopWatchModeInit(void);
void resetStopWatch(void);
void displayTime(void);


#endif /* OUTOFBOX_MSP430FR6989_STOPWATCHMODE_H_ */
