/*
 * gps.cpp
 *
 *  Created on: 26 Jul 2016
 *      Author: greg
 */


/*!

    \file   drvUART.cpp

    \brief  Atmega328p serial port driver
*/

#include "kerneltypes.h"
#include "gps.h"
#include "driver.h"
#include "thread.h"
#include "threadport.h"
#include "kerneltimer.h"

#include <fabooh.h>
#include <serial.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

serial_base_usci_t<9600, 4000000UL, GPS_RX, GPS_TX> Serial;

//---------------------------------------------------------------------------
static msp430GPS *pclActive;	// Pointer to the active object

//---------------------------------------------------------------------------
void msp430GPS::SetBaud(void)
{
}

//---------------------------------------------------------------------------
void msp430GPS::Init(void)
{

}

//---------------------------------------------------------------------------
K_UCHAR msp430GPS::Open()
{
	pclActive = this;
	Serial.begin(9600);

	UCA0IE = UCRXIE;
	return 0;
}

//---------------------------------------------------------------------------
K_UCHAR msp430GPS::Close(void)
{
	pclActive = (msp430GPS*)0;
    return 0;
}

//---------------------------------------------------------------------------
K_USHORT msp430GPS::Control( K_USHORT usCmdId_, void *pvIn_, K_USHORT usSizeIn_, void *pvOut_, K_USHORT usSizeOut_)
{
    switch ((CMD_GPS)usCmdId_)
    {
        case CMD_SET_BAUDRATE:
        {
        }
            break;
        case CMD_SET_BUFFERS:
        {
        }
            break;
        case CMD_SET_RX_ESCAPE:
        {
        }
            break;
        case CMD_SET_RX_CALLBACK:
        {
        	pfCallback = (UART_Rx_Callback_t)pvIn_;
        }
            break;
        case CMD_SET_RX_ECHO:
        {
        }
            break;
		case CMD_SET_RX_ENABLE:
		{
		}
			break;
		case CMD_SET_RX_DISABLE:
		{
		}
			break;
        default:
            break;
    }
    return 0;
}

//---------------------------------------------------------------------------
K_USHORT msp430GPS::Read( K_USHORT usSizeIn_, K_UCHAR *pvData_ )
{
    return m_ucTxSize;//fread( pvData_, 1, usSizeIn_, stderr );
}

//---------------------------------------------------------------------------
K_USHORT msp430GPS::Write(K_USHORT usSizeOut_, K_UCHAR *pvData_)
{
    return 0;//fwrite( pvData_, 1, usSizeOut_, stderr );
}

//---------------------------------------------------------------------------
void msp430GPS::StartTx(void)
{
    // stub
}

//---------------------------------------------------------------------------
void msp430GPS::RxISR()
{
	uint8_t rxChar = UCA0RXBUF;
	if(rxChar == '$')
		LED_B_BLUE::low();
	else if( rxChar == '\n')
		LED_B_BLUE::high();

	m_ucTxSize = rxChar;

	if( pfCallback != 0)
	{
		pfCallback(this);
	}
	// stub
}

//---------------------------------------------------------------------------
void msp430GPS::TxISR()
{
    // Stub
}

void __attribute__ ((__interrupt__(USCI_A0_VECTOR)))
usciA0_vect()
{
	switch(UCA0IV)
	{
	case USCI_UCRXIFG:
		if(pclActive)
			pclActive->RxISR();
		UCA0IFG &= ~UCRXIFG;
		break;
	case USCI_UCTXIFG: break;
	default: break;
	}
}

