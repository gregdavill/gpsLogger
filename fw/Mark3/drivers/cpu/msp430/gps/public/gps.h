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

    K_UCHAR m_ucTxSize;                //!< Size of the TX Buffer
    K_UCHAR m_ucTxHead;                //!< Head index
    K_UCHAR m_ucTxTail;                //!< Tail index

    K_UCHAR m_ucRxSize;                //!< Size of the RX Buffer
    K_UCHAR m_ucRxHead;                //!< Head index
    K_UCHAR m_ucRxTail;                //!< Tail index

    K_UCHAR m_bRxOverflow;              //!< Receive buffer overflow
    K_UCHAR m_bEcho;                    //!< Whether or not to echo RX characters to TX

    K_UCHAR *m_pucRxBuffer;            //!< Receive buffer pointer
    K_UCHAR *m_pucTxBuffer;            //!< Transmit buffer pointer

    K_ULONG m_ulBaudRate;              //!< Baud rate

    K_UCHAR m_ucRxEscape;              //!< Escape character

    UART_Rx_Callback_t    pfCallback;    //!< Callback function on matched escape character
};



#endif /* MARK3_DRIVERS_CPU_MSP430_GPS_PUBLIC_GPS_H_ */
