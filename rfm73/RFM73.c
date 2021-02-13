#include "RFM73.h"
#include "../utility/delay.h"

uint8_t op_status;
volatile uint8_t rx_value = 0;
bool rx_ready = false;
//Bank1 register initialization value
//In the array RegArrFSKAnalog,all the register value is the byte reversed!!!!!!!!!!!!!!!!!!!!!
const unsigned long Bank1_Reg0_13[]={       //latest config txt
0xE2014B40,
0x00004BC0,
0x028CFCD0,
0x41390099,
0x1B8296F9,
0xA60F0624,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00127300,
0x36B48000,
};

const unsigned long Bank1_Reg0_4[]={
0xDB8A96F9,
0x1B8296F9,
0xDB8296F9
};
const unsigned long Bank1_Reg0_5[]={
0xB60F0624,
0xA60F0624,
0xB60F0624
};

const uint8_t Bank1_Reg14[]=
{
    0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF
};

//Bank0 register initialization value
const uint8_t Bank0_Reg[][2]={
{0,0x0F},//reflect RX_DR\TX_DS\MAX_RT,Enable CRC ,2byte,POWER UP,PRX
{1,0x3F},//Enable auto acknowledgement data pipe5\4\3\2\1\0
{2,0x3F},//Enable RX Addresses pipe5\4\3\2\1\0
{3,0x03},//RX/TX address field width 5byte
{4,0xff},//auto retransmission dalay (4000us),auto retransmission count(15)
{5,0x17},//23 channel
{6,0x07},//air data rate-1M,out power 5dbm,setup LNA gain.
{7,0x07},//
{8,0x00},//
{9,0x00},
{12,0xc3},//only LSB Receive address data pipe 2, MSB bytes is equal to RX_ADDR_P1[39:8]
{13,0xc4},//only LSB Receive address data pipe 3, MSB bytes is equal to RX_ADDR_P1[39:8]
{14,0xc5},//only LSB Receive address data pipe 4, MSB bytes is equal to RX_ADDR_P1[39:8]
{15,0xc6},//only LSB Receive address data pipe 5, MSB bytes is equal to RX_ADDR_P1[39:8]
{17,0x20},//Number of bytes in RX payload in data pipe0(32 byte)
{18,0},//Number of bytes in RX payload in data pipe1(32 byte)
{19,0},//Number of bytes in RX payload in data pipe2(32 byte)
{20,0},//Number of bytes in RX payload in data pipe3(32 byte)
{21,0},//Number of bytes in RX payload in data pipe4(32 byte)
{22,0},//Number of bytes in RX payload in data pipe5(32 byte)
{23,0x00},//fifo status
{28,0x01},//Enable dynamic payload length data pipe5\4\3\2\1\0
{29,0x07}//Enables Dynamic Payload Length,Enables Payload with ACK,Enables the W_TX_PAYLOAD_NOACK command
};


const uint8_t RX0_Address[]={0x34,0x43,0x10,0x10,0x01};//Receive address data pipe 0
const uint8_t RX1_Address[]={0x39,0x38,0x37,0x36,0xc2};////Receive address data pipe 1



extern uint8_t test_data;
extern uint8_t  channel;
extern uint8_t  power;
extern uint8_t  data_rate;
extern uint8_t rx_buf[MAX_PACKET_LEN];


///////////////////////////////////////////////////////////////////////////////
//                  SPI access                                               //
///////////////////////////////////////////////////////////////////////////////

/**************************************************
Function: SPI_RW();

Description:
	Writes one uint8_t to RFM73, and return the uint8_t read
**************************************************/
uint8_t SPI_RW(uint8_t value)
{

    uint8_t bite_ctr;
    for(bite_ctr=0;bite_ctr<8;bite_ctr++)   // output 8-bit
    {
        if(value & 0x80)
        {
            //MOSI=1;
            GPIO_setOutputHighOnPin(
                2,
                BIT6
                );
        }
        else
        {
            //MOSI=0;
            GPIO_setOutputLowOnPin(
                2,
                BIT6
                );
        }

        value = (value << 1);           // shift next bit into MSB..
        //SCK = 1;                        // Set SCK high..
        GPIO_setOutputHighOnPin(
            2,
            BIT4
            );
        value |= GPIO_getInputPinValue(2, BIT5);                // capture current MISO bit
        GPIO_setOutputLowOnPin(
            2,
            BIT4
            );
        //SCK = 0;                      // ..then set SCK low again
    }
    return(value);                    // return read uint8_t




    volatile uint8_t temp = 0;
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;

    //Transmit Data to slave
    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, value);
    rx_ready = false;
    while(!rx_ready);   //rx_ready set in SPI ISR
    temp = rx_value;
    rx_value = 0;

    return temp;

//	uint8_t bit_ctr;
//	for(bit_ctr=0;bit_ctr<8;bit_ctr++)   // output 8-bit
//	{
//		if(value & 0x80)
//		{
//			MOSI=1;
//		}
//		else
//		{
//			MOSI=0;
//		}
//
//		value = (value << 1);           // shift next bit into MSB..
//		SCK = 1;						// Set SCK high..
//		value |= MISO;       		  // capture current MISO bit
//		SCK = 0;            		  // ..then set SCK low again
//	}
//	return(value);           		  // return read uint8_t
}

/**************************************************
Function: SPI_Write_Reg();

Description:
	Writes value 'value' to register 'reg'
/**************************************************/
void SPI_Write_Reg(uint8_t reg, uint8_t value)
{
    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

    SPI_RW(reg);      // select register
    SPI_RW(value);             // ..and write value to it..

//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//    //Transmit Data to slave
//    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, reg); // select register
//
//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//    //Transmit Data to slave
//    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, value); // ..and write value to it..

    GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );
}
/**************************************************/

/**************************************************
Function: SPI_Read_Reg();

Description:
	Read one uint8_t from BK2421 register, 'reg'
/**************************************************/
uint8_t SPI_Read_Reg(uint8_t reg)
{
	volatile uint8_t value;
    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

	SPI_RW(reg);            // Select register to read from..
	value = SPI_RW(0);    // ..then read register value

//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//    //Transmit Data to slave
//    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, reg); // select register
//
//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, 0x00); // dummy data
//
//    rx_ready = false;
//    while(!rx_ready);
//    value = rx_value;
//    rx_value = 0;

    GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

	return(value);        // return register value
}
/**************************************************/

/**************************************************
Function: SPI_Read_Buf();

Description:
	Reads 'length' #of length from register 'reg'
/**************************************************/
void SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
	uint8_t byte_ctr;

    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

	    SPI_RW(reg);       		// Select register to write, and read status uint8_t
//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//    //Transmit Data to slave
//    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, reg); // select register

	for(byte_ctr=0;byte_ctr<length;byte_ctr++){
		pBuf[byte_ctr] = SPI_RW(0);    // Perform SPI_RW to read uint8_t from RFM73
	}

    GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

}
/**************************************************/

/**************************************************
Function: SPI_Write_Buf();

Description:
	Writes contents of buffer '*pBuf' to RFM73
/**************************************************/
void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
	uint8_t byte_ctr;

    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

	SPI_RW(reg);    // Select register to write to and read status uint8_t
	for(byte_ctr=0; byte_ctr<length; byte_ctr++){ // then write all uint8_t in buffer(*pBuf)
		SPI_RW(*pBuf++);
//	    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
//	              EUSCI_A_SPI_TRANSMIT_INTERRUPT)) ;
//
//	    //Transmit Data to slave
//	    EUSCI_A_SPI_transmitData(EUSCI_A1_BASE, *pBuf++); // select register
	}
    GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CS,
        GPIO_PIN_RFM_CS
        );

}
/**************************************************
Function: SwitchToRxMode();
Description:
	switch to Rx mode
**************************************************/
void SwitchToRxMode()
{
	uint8_t value;

	SPI_Write_Reg(FLUSH_RX,0);//flush Rx

	value=SPI_Read_Reg(STATUS);	// read register STATUS's value
	SPI_Write_Reg(WRITE_REG|STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CE,
        GPIO_PIN_RFM_CE
        );

	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value

//PRX
	value=value|0b00000011;//set bit 1
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

  	GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CE,
        GPIO_PIN_RFM_CE
        );
}

/**************************************************
Function: SwitchToTxMode();
Description:
	switch to Tx mode
**************************************************/
void SwitchToTxMode()
{
	uint8_t value;
	SPI_Write_Reg(FLUSH_TX,0);//flush Tx

    GPIO_setOutputLowOnPin(
        GPIO_PORT_RFM_CE,
        GPIO_PIN_RFM_CE
        );
	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	value |= BIT1;//set bit 0
	value &= ~BIT0;
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.

    GPIO_setOutputHighOnPin(
        GPIO_PORT_RFM_CE,
        GPIO_PIN_RFM_CE
        );
}

/**************************************************
Function: SwitchCFG();

Description:
	 access switch between Bank1 and Bank0

Parameter:
	_cfg      1:register bank1
	          0:register bank0
Return:
     None
**************************************************/
void SwitchCFG(char _cfg)//1:Bank1 0:Bank0
{
	uint8_t Tmp;

	Tmp=SPI_Read_Reg(7);
	Tmp=Tmp&0x80;

	if( ( (Tmp)&&(_cfg==0) )
	||( ((Tmp)==0)&&(_cfg) ) )
	{
		SPI_Write_Reg(ACTIVATE_CMD,0x53);
	}
}

/**************************************************
Function: SetChannelNum();
Description:
	set channel number

**************************************************/
void SetChannelNum(uint8_t ch)
{
	SPI_Write_Reg((uint8_t)(WRITE_REG|5),(uint8_t)(ch));
}


///////////////////////////////////////////////////////////////////////////////
//                  RFM73 initialization                                    //
///////////////////////////////////////////////////////////////////////////////
/**************************************************
Function: RFM73_Initialize();

Description:
    register initialization
**************************************************/
void RFM73_Initialize()
{
    uint8_t i,j,temp;
    uint8_t WriteArr[12];

    const uint8_t channel = 0;
    const uint8_t data_rate = 0;
    const uint8_t power = 2;

    DelayMs(100);//delay more than 50ms.

    SwitchCFG(0);
    //**************************Test spi*****************************//
    //SPI_Write_Reg((WRITE_REG|Bank0_Reg[3][0]),0x04);
    //test_data = SPI_Read_Reg(3);

//********************Write Bank0 register******************

    for(i=0;i<5;i++)
    {
        SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);
    }

//********************select channel*************************
    switch(channel)
    {
        case 0:

            SPI_Write_Reg(WRITE_REG|5,0x0a); //channel 2.410G
            break;

        case 1:

            SPI_Write_Reg(WRITE_REG|5,0x1e); //channel 2.430G
            break;
        case 2:

            SPI_Write_Reg(WRITE_REG|5,0x3c); //channel 2.460G
            break;

        case 3:

            SPI_Write_Reg(WRITE_REG|5,0x53); //channel 2.483G
            break;

        default:
            break;
    }
    if(data_rate)
    {
        temp = 0x28;      //data rate 2M
    }
    else
    {
        temp = 0x00;     //data rate 1M
    }

    switch(power)
    {
        case 0:

            temp |= 0x01;
            SPI_Write_Reg(WRITE_REG|6,temp); //power -7dbm
            break;

        case 1:

            temp |= 0x03;
            SPI_Write_Reg(WRITE_REG|6,temp); //power -3dbm
            break;
        case 2:

            temp |= 0x05;
            SPI_Write_Reg(WRITE_REG|6,temp); //power 0dbm
            break;

        case 3:

            temp |= 0x07;
            SPI_Write_Reg(WRITE_REG|6,temp); //power 5dbm
            break;

        default:
            break;
    }
    for(i=7;i<21;i++)
    {
        SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);
    }
/************************************************************/
//reg 10 - Rx0 addr
//    for(j=0;j<5;j++)
//    {
//        WriteArr[j]=RX0_Address[j];
//    }
//    SPI_Write_Buf((WRITE_REG|10),&(WriteArr[0]),5);
//
////REG 11 - Rx1 addr
//    for(j=0;j<5;j++)
//    {
//        WriteArr[j]=RX1_Address[j];
//    }
//    SPI_Write_Buf((WRITE_REG|11),&(WriteArr[0]),5);
////REG 16 - TX addr
//    for(j=0;j<5;j++)
//    {
//        WriteArr[j]=RX0_Address[j];
//    }
//    SPI_Write_Buf((WRITE_REG|16),&(WriteArr[0]),5);


    i=SPI_Read_Reg(29);//read Feature Register 如果要支持动态长度或者 Payload With ACK，需要先给芯片发送 ACTIVATE命令（数据为0x73),然后使能动态长度或者 Payload With ACK (REG28,REG29).
    if(i==0) // i!=0 showed that chip has been actived.so do not active again.
        SPI_Write_Reg(ACTIVATE_CMD,0x73);// Active
    for(i=22;i>=21;i--)
    {
        SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);//Enable Dynamic Payload length ,Enables the W_TX_PAYLOAD_NOACK command
    }

//********************Write Bank1 register******************
    SwitchCFG(1);

    //verify chip version
    SPI_Read_Buf(0x08, WriteArr, 4);

    if(WriteArr[0] != 0x63){

        __delay_cycles(1);

    }

//  for(i=0;i<=8;i++)//reverse
//  {
//      for(j=0;j<4;j++)
//          WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;
//
//      SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);
//  }
    for(i=0;i<=3;i++)//reverse
    {
        for(j=0;j<4;j++)
            WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;

        SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);
    }

    for(j=0;j<4;j++)
        WriteArr[j]=(Bank1_Reg0_4[data_rate+1]>>(8*(j) ) )&0xff;
    SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);
    for(j=0;j<4;j++)
        WriteArr[j]=(Bank1_Reg0_5[data_rate+1]>>(8*(j) ) )&0xff;
    SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);

    for(i=6;i<=8;i++)//reverse
    {
        for(j=0;j<4;j++)
            WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;

        SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);
    }

    for(i=9;i<=13;i++)
    {
        for(j=0;j<4;j++)
            WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(3-j) ) )&0xff;

        SPI_Write_Buf((WRITE_REG|i),&(WriteArr[0]),4);
    }

    for(j=0;j<11;j++)
    {
        WriteArr[j]=Bank1_Reg14[j];
    }
    SPI_Write_Buf((WRITE_REG|14),&(WriteArr[0]),11);

//toggle REG4<25,26>
    for(j=0;j<4;j++)
        WriteArr[j]=(Bank1_Reg0_13[4]>>(8*(j) ) )&0xff;

    WriteArr[0]=WriteArr[0]|0x06;
    SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);

    WriteArr[0]=WriteArr[0]&0xf9;
    SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);

    DelayMs(10);

//********************switch back to Bank0 register access******************
    SwitchCFG(0);

    SwitchToRxMode();//switch to RX mode
    //SwitchToTxMode();//switch to RX mode
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_SPI_UCRXIFG:      // UCRXIFG
            //USCI_A0 TX buffer ready?
            while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A1_BASE,
                        EUSCI_A_SPI_TRANSMIT_INTERRUPT));

            rx_value = EUSCI_A_SPI_receiveData(EUSCI_A1_BASE);

            rx_ready = true;

            __delay_cycles(40);
            break;
        default:
            break;
    }
}

void DelayMs(uint16_t ms){
    delay_ms(ms);
}

