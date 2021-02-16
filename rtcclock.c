/*
 * rtcclock.c
 *
 *  Created on: Nov 19, 2017
 *      Author: spiri
 */

#include "rtcclock.h"
#include <msp430.h>
#include <stdlib.h>
#include <driverlib.h>

// The use of unsigned 32-bit timestamps supports 136-year operation, from 1970 to 2106.
// Hopefully we're not still using tiny 16-bit MCUs by then...

// Statically-initialized variable
#ifdef __TI_COMPILER_VERSION__
#pragma PERSISTENT(rtcclock_current)
uint64_t rtcclock_current = 0;
#elif __IAR_SYSTEMS_ICC__
__persistent uint64_t rtcclock_current = 0;
#elif defined(__GNUC__)
uint64_t rtcclock_current __attribute__((section(".text"))) = 0;
#endif

/* Database of current alarms */
rtclock_alarm_t * rtcAlarms[RTC_MAX_ALARM_COUNT];

// Use SMCLK for those without a soldered XT1 who go no lower than LPM0
void RTClock_init(uint64_t curclk, uint32_t  interval)
{
    if (curclk != 0) {
        // Write curclk to rtcclock_current
        SYSCFG0 = FRWPPW | DFWP;
        rtcclock_current = curclk;
        SYSCFG0 = FRWPPW | PFWP | DFWP;
    }
    //Initialize RTC
    RTC_init(RTC_BASE,
        interval,
        RTC_CLOCKPREDIVIDER_1);

    RTC_clearInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT_FLAG);

    //Enable interrupt for RTC overflow
    RTC_enableInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT);

    //Start RTC Clock with clock source SMCLK
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_XT1CLK);
}

// RTC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR (void)
{
    //P1OUT ^= BIT0;
    uint16_t frwp = SYSCFG0 & (DFWP | PFWP); // Save SYSCFG0 so we restore it before exiting

    SYSCFG0 = FRWPPW;
    rtcclock_current++;
    unsigned int i;
    for (i=RTC_MAX_ALARM_COUNT; i > 0; i--) {
        if (rtcAlarms[i-1] != (void *)0) {
            if (rtcAlarms[i-1]->timestamp == rtcclock_current) {
                // We can support FRAM-based rtc_alarm_t struct pointers b/c FRAM WP is disabled earlier in this ISR.
                rtcAlarms[i-1]->triggered = true;
                __bic_SR_register_on_exit(LPM0_bits);
            }
        }
    }
    SYSCFG0 = FRWPPW | frwp; // Restore SYSCFG0 to pre-interrupt state
    RTC_clearInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT_FLAG);
}

/* The bulk of this codebase is the interpretation functionality. */
int RTClock_compare(uint64_t tstmp)
{
    uint64_t tmpstamp = rtcclock_current; // Save current timestamp locally in case it changes mid-function

    if (tstmp > tmpstamp) {
        return 1;
    }
    if (tstmp < tmpstamp) {
        return -1;
    }
    return 0; // Exact same time!
}

void RTClock_get(uint64_t * buf)
{
    if (buf == (void *)0) {
        return;
    }

    *buf = rtcclock_current;
}

void RTClock_set(uint64_t new_value){
    SYSCFG0 = FRWPPW | DFWP;
    rtcclock_current = new_value;
    SYSCFG0 = FRWPPW | PFWP | DFWP;
}

bool RTClock_setAlarm(rtclock_alarm_t * a)
{
    unsigned int i;

    for (i=RTC_MAX_ALARM_COUNT; i > 0; i--) {
        if (rtcAlarms[i-1] == (void *)0) {
            a->triggered = false;
            rtcAlarms[i-1] = a;
            return true; // Found an open slot for this alarm
        }
    }
    return false; // Ran out of slots to put this in
}

bool RTClock_clearAlarm(rtclock_alarm_t * a)
{
    unsigned int i;

    for (i=RTC_MAX_ALARM_COUNT; i > 0; i--) {
        if (rtcAlarms[i-1] == a) {
            rtcAlarms[i-1] = (void *)0;
            return true; // Found & cleared the alarm
        }
    }
    return false; // Didn't find it
}
