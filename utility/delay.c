/*
 * delay.c
 *
 *  Created on: Dec 14, 2016
 *      Author: mark
 */


#include "delay.h"


void delay_sec(unsigned short sec) {
    for (;sec > 0; sec--) {
        delay_ms(1000);
    }
}


void delay_ms(volatile unsigned short p) {
    for (;p > 0; p--) {
        __delay_cycles(CONFIG_CLOCK_FREQUENCY * DELAY_CLOCK_MULTIPLIER);
    }
}

inline void delay_us(volatile unsigned short p) {
    for (;p > 0; p--) {
        __delay_cycles(CONFIG_CLOCK_FREQUENCY);
    }
}
