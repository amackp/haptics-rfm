/*********************************************************
copyright(c) 2012
Title: 				RFM73 simple example based on PIC c
Current 			version: v1.0
Function:			RFM73 demo
processor: 			PIC16F690
Clock:				internal RC 8M
Author:				baiguang(ISA)
Company:			Hope microelectronic Co.,Ltd.
Contact:			+86-0755-82973805-846
E-MAIL:				rfeng@hoperf.com
Data:				2012-11-10
**********************************************************/
/*********************************************************
                 ---------------
                |VDD         VSS|
 	IRQ	----    |RA5         RA0|	 ----CE
    MOSI----    |RA4         RA1|	 ----CSN
    MISO----    |RA3         RA2|    ----SCK
        ----    |RC5         RC0|    ----
        ----    |RC4         RC1|	 ----
        ----    |RC3         RC2|    ----
        ----    |RC6         RB4|    ----
 	    ----    |RC7         RB5|	 ----
        ----	|RB7         RB6|	 ----
			     ---------------
					pic16F690
*********************************************************/

/*--------------------------------------------------------------------------------------
*This file is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
*****************
*MCU: PIC16f690
Compiler:MPLAB8.10
*****************
* website: http://www.hoperf.com
---------------------------------------------------------------------------------------*/
#ifndef _RFM73_H_
#define _RFM73_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <msp430.h>
#include <driverlib.h>
#include "../Board.h"
#define TRUE 1
#define FALSE 0

#define CE   BIT2
#define CE_PORT P1OUT
#define CSN  BIT3
#define CSN_PORT P1OUT
#define SCK  BIT7
#define SCK_PORT P2OUT
#define MISO BIT7
#define MISO_PORT P1IN
#define MOSI BIT4
#define MOSI_PORT P2OUT
#define IRQ  BIT6
#define IRQ_PORT P1IN



#define MAX_PACKET_LEN  32// max value is 32


//************************FSK COMMAND and REGISTER****************************************//
// SPI(RFM73) commands
#define READ_REG        		0x00  // Define read command to register
#define WRITE_REG       		0b00100000  // Define write command to register
#define RD_RX_PLOAD     		0b01100001  // Define RX payload register address
#define WR_TX_PLOAD     		0b10100000  // Define TX payload register address
#define FLUSH_TX        		0b11100001  // Define flush TX register command
#define FLUSH_RX        		0b11100010  // Define flush RX register command
#define REUSE_TX_PL     		0b11100011  // Define reuse TX payload register command
#define W_TX_PAYLOAD_NOACK_CMD	0b10100000
#define W_ACK_PAYLOAD_CMD		0b10101000
#define ACTIVATE_CMD			0b01010000
#define R_RX_PL_WID_CMD			0b01100000
#define NOP_NOP            		0xFF  // Define No Operation, might be used to read status register

// SPI(RFM73) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define PAYLOAD_WIDTH   0x1f  // 'payload length of 256 bytes modes register address

//interrupt status
#define STATUS_RX_DR 	0x40
#define STATUS_TX_DS 	0x20
#define STATUS_MAX_RT 	0x10

#define STATUS_TX_FULL 	0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE 	0x40
#define FIFO_STATUS_TX_FULL 	0x20
#define FIFO_STATUS_TX_EMPTY 	0x10

#define FIFO_STATUS_RX_FULL 	0x02
#define FIFO_STATUS_RX_EMPTY 	0x01


uint8_t SPI_Read_Reg(uint8_t reg);
void SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

void SPI_Write_Reg(uint8_t reg, uint8_t value);
//void SPI_Write_Reg_Bank0(const uint8_t reg, const uint8_t value);
//void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length);
void SPI_Write_Buf_bank(uint8_t reg, uint8_t *pBuf, uint8_t length);


void SwitchToTxMode(void);
void SwitchToRxMode(void);

void SPI_Bank1_Read_Reg(uint8_t reg, uint8_t *pBuf);
void SPI_Bank1_Write_Reg(uint8_t reg, uint8_t *pBuf);
void SwitchCFG(char _cfg);

void RFM73_Initialize(void);


void DelayMs(uint16_t ms);

#endif
