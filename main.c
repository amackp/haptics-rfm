/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
//******************************************************************************
//!
//! RTC in Counter Mode toggles LED1 every 1s
//!
//!  This program demonstrates RTC in counter mode configured to source from SMCLK
//!  to toggle LED1 every 1s.
//!  SMCLK = REFO = ~32kHz
//!
//!           MSP430FR2xx_4xx Board
//!             -----------------
//!        /|\ |                 |
//!         |  |                 |
//!         ---|RST              |
//!            |                 |
//!            |                 |-->LED1
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - RTC peripheral
//! - GPIO Port peripheral
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - RTC_VECTOR
//!
//******************************************************************************
#include "driverlib.h"
#include "Board.h"
#include <stdint.h>
#include <msp430.h>
#include "rtcclock.h"
#include "rfm73/RFM73.h"
#include "utility/delay.h"
//*****************************************************************************
//
//Interval time used for RTC
//
//*****************************************************************************
#define TIME_INTERVAL_MS    10
#define INTERVAL_TIME       (1000000/(1000/TIME_INTERVAL_MS))
#define TIME_1S             (1000/TIME_INTERVAL_MS)

#define TIME_10HZ           (TIME_1S/10)
#define TIME_5HZ            (TIME_1S/5)
#define TIME_1HZ            (TIME_1S)
#define TIME_0_5HZ          (TIME_1S*2)
#define TIME_0_1HZ          (TIME_1S*10)



typedef enum{
    FREQ_10HZ = 0,
    FREQ_5HZ,
    FREQ_1HZ,
    FREQ_0_5HZ,
    FREQ_0_1HZ,

    FREQ_NULL
}freq_selection_t;

uint16_t freq_map[FREQ_NULL] = {
                                TIME_10HZ,
                                TIME_5HZ,
                                TIME_1HZ,
                                TIME_0_5HZ,
                                TIME_0_1HZ
                                };

freq_selection_t current_freq = FREQ_1HZ;

rtclock_alarm_t led_alarm, alarm_1s;
uint64_t alarm_time = 0;

void reset_alarm(rtclock_alarm_t * alarm, uint16_t interval){
    RTClock_get(&alarm_time);
    alarm->timestamp = alarm_time + interval;
    RTClock_setAlarm(alarm);
}

void clock_init(){
    CS_initClockSignal(
        CS_FLLREF,
        CS_REFOCLK_SELECT,
        CS_CLOCK_DIVIDER_1
        );


    //Set DCO FLL reference = REFO
    CS_initFLLSettle(8000, 244);



    //Set SMCLK = REFO
    CS_initClockSignal(
        CS_ACLK,
        CS_REFOCLK_SELECT,
        CS_CLOCK_DIVIDER_1
        );
    CS_initClockSignal(
        CS_SMCLK,
        CS_DCOCLKDIV_SELECT,
        CS_CLOCK_DIVIDER_8
        );
    CS_initClockSignal(
        CS_MCLK,
        CS_DCOCLKDIV_SELECT,
        CS_CLOCK_DIVIDER_1
        );

    volatile uint32_t m_freq = CS_getMCLK();
    volatile uint32_t sm_freq = CS_getSMCLK();
    volatile uint32_t a_feq = CS_getACLK();

    __delay_cycles(1);

}

void gpio_init(){
    //Set LED1 to output direction
    GPIO_setAsOutputPin(
        GPIO_PORT_LED1,
        GPIO_PIN_LED1 + GPIO_PIN_LED2 + BIT5
        );
    GPIO_setAsOutputPin(
            GPIO_PORT_RFM_CS,
            GPIO_PIN_RFM_CS
        );
    GPIO_setAsOutputPin(
            GPIO_PORT_RFM_CE,
            GPIO_PIN_RFM_CE
        );
    GPIO_setAsInputPinWithPullUpResistor(
        2,
        BIT3+BIT7
        );
    GPIO_enableInterrupt(
        2,
        BIT3+BIT7
        );
    GPIO_selectInterruptEdge(2, BIT3+BIT7, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(2, 0xff);

    GPIO_setAsOutputPin(
            2,
            BIT4 + BIT6
            );
    GPIO_setAsInputPin(
            2,
            BIT5
            );

    P2OUT |= BIT4 + BIT6;

//    GPIO_setAsPeripheralModuleFunctionInputPin(
//        GPIO_PORT_P2,
//        GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6,
//        GPIO_PRIMARY_MODULE_FUNCTION
//    );
//
//    //Initialize Master
//    EUSCI_A_SPI_initMasterParam param = {0};
//    param.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK;
//    param.clockSourceFrequency = CS_getSMCLK();
//    param.desiredSpiClock = 100000;
//    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
//    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
//    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
//    param.spiMode = EUSCI_A_SPI_3PIN;
//    EUSCI_A_SPI_initMaster(EUSCI_A1_BASE, &param);
//
//    //Enable SPI module
//    EUSCI_A_SPI_enable(EUSCI_A1_BASE);
//
//    //Clear receive interrupt flag
//    EUSCI_A_SPI_clearInterrupt(EUSCI_A1_BASE,
//          EUSCI_A_SPI_RECEIVE_INTERRUPT
//          );
//
//    // Enable USCI_A0 RX interrupt
//    EUSCI_A_SPI_enableInterrupt(EUSCI_A1_BASE,
//                         EUSCI_A_SPI_RECEIVE_INTERRUPT);
//

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();
}

void sub_program_1hz(void);

void Send_Packet(uint8_t type,uint8_t* pbuf,uint8_t len);
void Send_NACK_Packet(void);
void Receive_Packet(void);
void SPI_Bank1_Write_Reg(uint8_t reg, uint8_t *pBuf);
void SPI_Bank1_Read_Reg(uint8_t reg, uint8_t *pBuf);
void Carrier_Test(uint8_t b_enable); //carrier test

extern void RFM73_Initialize(void);
extern void SwitchToTxMode(void);
extern void SwitchToRxMode(void);

const uint8_t tx_buf[17]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x78};
uint8_t rx_buf[MAX_PACKET_LEN];

extern const uint8_t RX0_Address[];
extern const unsigned long Bank1_Reg0_13[];

uint8_t test_data;

void rfm_master()
{
    //sends current clock and freq mode to slave
    uint8_t i;
    uint8_t temp_buf[9];
    uint64_t curr_clock = 0;
    RTClock_get(&curr_clock);

    temp_buf[0] = (curr_clock >> 56) & 0xff;
    temp_buf[1] = (curr_clock >> 48) & 0xff;
    temp_buf[2] = (curr_clock >> 40) & 0xff;
    temp_buf[3] = (curr_clock >> 32) & 0xff;
    temp_buf[4] = (curr_clock >> 24) & 0xff;
    temp_buf[5] = (curr_clock >> 16) & 0xff;
    temp_buf[6] = (curr_clock >> 8) & 0xff;
    temp_buf[7] = curr_clock & 0xff;
    temp_buf[8] = current_freq;

    Send_Packet(WR_TX_PLOAD,temp_buf,9);
    SwitchToRxMode();  //switch to Rx mode
}

void rfm_slave(){
    Receive_Packet();
}

/*********************************************************
Function:  sub_program_1hz()

Description:

*********************************************************/
void sub_program_1hz(void)
{
    uint8_t i;
    uint8_t temp_buf[32];

    for(i=0;i<32;i++)
    {
        temp_buf[i]=0x00;
    }

    Send_Packet(WR_TX_PLOAD,temp_buf,10);
    SwitchToRxMode();  //switch to Rx mode
}
/**************************************************
Function: Send_Packet
Description:
    fill FIFO to send a packet
Parameter:
    type: WR_TX_PLOAD or  W_TX_PAYLOAD_NOACK_CMD
    pbuf: a buffer pointer
    len: packet length
Return:
    None
**************************************************/
void Send_Packet(uint8_t type,uint8_t* pbuf,uint8_t len)
{
    uint8_t fifo_sta;
    SwitchToTxMode();

    fifo_sta=SPI_Read_Reg(FIFO_STATUS); // read register FIFO_STATUS's value
    if(!(fifo_sta&BIT5))//if not full, send data (write buff)
    {
        SPI_Write_Buf(type, pbuf, len); // Writes data to buffer
        GPIO_setOutputHighOnPin(
                GPIO_PORT_LED2,
                GPIO_PIN_LED2);
        delay_ms(100);
        GPIO_setOutputLowOnPin(
                GPIO_PORT_LED2,
                GPIO_PIN_LED2);
    }
}
/**************************************************
Function: Receive_Packet
Description:
    read FIFO to read a packet
Parameter:
    None
Return:
    None
**************************************************/
void Receive_Packet(void)
{
    uint8_t  len,i,j,k,sta,fifo_sta,value,chksum;
    uint64_t new_clock = 0;

    //SwitchToRxMode();

    sta=SPI_Read_Reg(STATUS); // read register STATUS's value

    __delay_cycles(1);
    if((STATUS_RX_DR&sta) == 0x40)        // if receive data ready (RX_DR) interrupt
    {
      do
      {
        len=SPI_Read_Reg(R_RX_PL_WID_CMD);  // read len

        if(len<=MAX_PACKET_LEN)
        {
          SPI_Read_Buf(RD_RX_PLOAD,rx_buf,len);// read receive payload from RX_FIFO buffer
          GPIO_setOutputHighOnPin(
                  GPIO_PORT_LED2,
                  GPIO_PIN_LED2);
        }
        else
        {
          SPI_Write_Reg(FLUSH_RX,0);              //flush Rx
        }

        fifo_sta=SPI_Read_Reg(FIFO_STATUS); // read register FIFO_STATUS's value

      }while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty

      //length for test packet
      if(len == 9){
          GPIO_setOutputHighOnPin(
                  GPIO_PORT_LED2,
                  GPIO_PIN_LED2);
          uint64_t *temp_p;
          temp_p = (uint64_t*)rx_buf;

          new_clock = *temp_p;
          current_freq = rx_buf[8];

          RTClock_set(new_clock);

          reset_alarm(&led_alarm, freq_map[current_freq]);
          reset_alarm(&alarm_1s, TIME_1S);
          GPIO_setOutputLowOnPin(
              GPIO_PORT_LED1,
              GPIO_PIN_LED1
              );
      }



//      while(1){
//          for(j = 0; j<len; j++){
//              if(rx_buf[j] == 0xab){
//                 for(k = 0; k < j+1; k++){
//                     GPIO_setOutputHighOnPin(
//                             GPIO_PORT_LED2,
//                             GPIO_PIN_LED2);
//                     delay_ms(500);
//                     GPIO_setOutputLowOnPin(
//                             GPIO_PORT_LED2,
//                             GPIO_PIN_LED2);
//                     delay_ms(500);
//                 }
//
//                 break;
//              }
//          }
//          break;
//      }



//      GPIO_setOutputHighOnPin(
//              GPIO_PORT_LED2,
//              GPIO_PIN_LED2);
      delay_ms(100);
      GPIO_setOutputLowOnPin(
              GPIO_PORT_LED2,
              GPIO_PIN_LED2);


      //Send_NACK_Packet();
      //Send_Packet(W_TX_PAYLOAD_NOACK_CMD,rx_buf,17);
      SwitchToRxMode();//switch to RX mode
      }
    SPI_Write_Reg(WRITE_REG|STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
}

volatile bool send_sync_message = false;

void main (void)
{
    WDT_A_hold(WDT_A_BASE);

    gpio_init();
    clock_init();
    RTClock_init(0, INTERVAL_TIME);

    reset_alarm(&led_alarm, freq_map[current_freq]);
    reset_alarm(&alarm_1s, TIME_1S);

    __enable_interrupt();

    RFM73_Initialize();
    SwitchToRxMode();  //switch to tx mode

    static const bool master = 0;

    //master_program
    while(master){
        if(led_alarm.triggered){
            led_alarm.triggered = false;
            reset_alarm(&led_alarm, freq_map[current_freq]);
            GPIO_toggleOutputOnPin(
                    GPIO_PORT_LED1,
                    GPIO_PIN_LED1);
        }
        if(alarm_1s.triggered){
            alarm_1s.triggered = false;
            reset_alarm(&alarm_1s, TIME_1S);
//            GPIO_toggleOutputOnPin(
//                    GPIO_PORT_LED2,
//                    GPIO_PIN_LED2 + BIT5);
           // Receive_Packet();
        }

        if(send_sync_message){
            send_sync_message = false;
            rfm_master();
            reset_alarm(&led_alarm, freq_map[current_freq]);
            reset_alarm(&alarm_1s, TIME_1S);
            GPIO_setOutputLowOnPin(
                GPIO_PORT_LED1,
                GPIO_PIN_LED1
                );
        }
    }

    //slave_program
    while(!master){
        if(led_alarm.triggered){
            led_alarm.triggered = false;
            reset_alarm(&led_alarm, freq_map[current_freq]);
            GPIO_toggleOutputOnPin(
                    GPIO_PORT_LED1,
                    GPIO_PIN_LED1);
        }
        if(alarm_1s.triggered){
            alarm_1s.triggered = false;
            reset_alarm(&alarm_1s, TIME_1S);
//            GPIO_toggleOutputOnPin(
//                    GPIO_PORT_LED2,
//                    GPIO_PIN_LED2 + BIT5);
            //sub_program_1hz();  //not sure if required

        }
        Receive_Packet();

//        if(send_sync_message){
//            rfm_master();
//        }
    }


    //Enter LPM3 mode with interrupts enabled
//    __bis_SR_register(LPM0_bits + GIE);
//    //__enable_interrupt();
//    __no_operation();
    //unsigned long clock_speed = CS_getMCLK();
}

//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=RTC_VECTOR
//__interrupt
//#elif defined(__GNUC__)
//__attribute__((interrupt(RTC_VECTOR)))
//#endif
//void RTC_ISR (void)
//{
//    switch (__even_in_range(RTCIV,2)){
//        case 0: break;  //No interrupts
//        case 2:         //RTC overflow
//            rtc_milis++;
//            if(rtc_milis == 1000){
//                rtc_milis = 0;
//                rtc_counter++;
//                GPIO_toggleOutputOnPin(
//                        GPIO_PORT_LED1,
//                        GPIO_PIN_LED1);
//            }
//            break;
//        default: break;
//    }
//}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT2_VECTOR)))
#endif
void PORT2_ISR(void)
{
    if(P2IFG&BIT3){
        send_sync_message = true;
    }
    else if(P2IFG&BIT7){
        current_freq = (current_freq + 1) % FREQ_NULL;
        reset_alarm(&led_alarm, freq_map[current_freq]);
        GPIO_setOutputLowOnPin(
            GPIO_PORT_LED1,
            GPIO_PIN_LED1
            );
    }
    GPIO_clearInterrupt(2, 0xff);
}
