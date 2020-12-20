/*
 * delay.h
 *
 *  Created on: Jan 7, 2020
 *      Author: Sanchez
 */

#ifndef DELAY_H_
#define DELAY_H_

#define SMCLK_F 4000000
//#define USE_DUMMY_DELAY


void delay_s (uint16_t);
void delay_ms(uint16_t);
void delay_us(uint16_t);
#endif /* DELAY_H_ */
