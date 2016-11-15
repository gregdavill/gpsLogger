/*
 * gps.h
 *
 *  Created on: 26 Jul 2016
 *      Author: greg
 */
/*!

    \file   drvUART.h

    \brief  Atmega328p serial port driver

*/


#ifndef MARK3_DRIVERS_CPU_MSP430_GPS_PUBLIC_GPS_H_
#define MARK3_DRIVERS_CPU_MSP430_GPS_PUBLIC_GPS_H_


#include "kerneltypes.h"
#include "driver.h"

#include <fabooh.h>


#define UART_DEFAULT_BAUD       ((K_ULONG)9600)


//---------------------------------------------------------------------------
typedef enum
{
    CMD_SET_BAUDRATE = 0x80,
    CMD_SET_BUFFERS,
    CMD_SET_RX_ESCAPE,
    CMD_SET_RX_CALLBACK,
    CMD_SET_RX_ECHO,
	CMD_SET_RX_ENABLE,
	CMD_SET_RX_DISABLE,
} CMD_GPS;

class msp430GPS;
//---------------------------------------------------------------------------
typedef struct _UartData_t UartData_t;

//---------------------------------------------------------------------------
typedef void (*UART_Rx_Callback_t)( msp430GPS *pclUART );

//---------------------------------------------------------------------------
/*!
	Implements a UART driver on the ATMega328p
*/
class msp430GPS : public Driver
{

public:
	virtual void Init();
	virtual K_UCHAR Open();
	virtual K_UCHAR Close();
	virtual K_USHORT Read( K_USHORT usBytes_,
								 K_UCHAR *pucData_ );

	virtual K_USHORT Write( K_USHORT usBytes_,
								  K_UCHAR *pucData_ );

	virtual K_USHORT Control( K_USHORT usEvent_,
									void *pvIn_,
									K_USHORT usSizeIn_,
									void *pvOut_,
									K_USHORT usSizeOut_ );
	/*!
		Called from the transmit complete ISR - implements a
		callback/transmit state-machine
	*/
	void TxISR();

	/*!
		Called from the receive-complete ISR - implements a
		callback/receive state-machine
	*/
	void RxISR();



private:

	void SetBaud(void);
	void StartTx(void);


	ringbuffer_t<uint8_t, 16, uint8_t> pRingBuffer;
	ringbuffer_t<uint8_t, 4, uint8_t> pTxBuffer;

    K_UCHAR m_ucRxEscape;              //!< Escape character

    UART_Rx_Callback_t    pfCallback;    //!< Callback function on matched escape character
};



#endif /* MARK3_DRIVERS_CPU_MSP430_GPS_PUBLIC_GPS_H_ */
