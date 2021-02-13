/*
 * delay.h
 *
 *  Created on: Nov 30, 2016
 *      Author: mark
 */

#ifndef SYSTEM_UTILITY_DELAY_H_
#define SYSTEM_UTILITY_DELAY_H_

#define CONFIG_CLOCK_FREQUENCY      8
#define DELAY_CLOCK_MULTIPLIER      1000


void delay_us(volatile unsigned short p);

void delay_ms(volatile unsigned short p);

void delay_sec(unsigned short sec);


#endif /* SYSTEM_UTILITY_DELAY_H_ */
