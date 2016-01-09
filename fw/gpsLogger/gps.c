/*
 * gps.c
 *
 *  Created on: 29 Dec 2015
 *      Author: Gregory
 */

#include <msp430.h>
#include "driverlib.h"

#include "USB_app/FatFs/ff.h"

volatile uint8_t* gps_current_buffer;
volatile uint8_t* gps_full_buffer;
volatile uint8_t gps_rx_bufferA[256];
volatile uint8_t gps_rx_bufferB[256];
volatile uint8_t gps_rx_idx;
volatile uint8_t gps_got_line;


FATFS FatFs;		/* FatFs work area needed for each volume */
FIL gps_log;			/* File object needed for each open file */



uint8_t bActive;
/* Use UCA1 */
void GPS_init()
{
	 //Configure UART pins (UCA1TXD/UCA1SIMO, UCA1RXD/UCA1SOMI)
	    //Set P4.4 and P4.5 as Module Function Input
	    GPIO_setAsPeripheralModuleFunctionInputPin(
	        GPIO_PORT_P4,
	        GPIO_PIN4 + GPIO_PIN5
	        );

	    //Baudrate = 9600, clock freq = 1.048MHz
	        //UCBRx = 109, UCBRFx = 0, UCBRSx = 2, UCOS16 = 0
	    USCI_A_UART_initParam param = {0};
	    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
	        param.clockPrescalar = 866;
	        param.firstModReg = 0;
	        param.secondModReg = 2;
	        param.parity = USCI_A_UART_NO_PARITY;
	        param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
	        param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	        param.uartMode = USCI_A_UART_MODE;
	        param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

	    if(STATUS_FAIL == USCI_A_UART_init(USCI_A1_BASE, &param))
	    {
	        return;
	    }

	    USCI_A_UART_enable(USCI_A1_BASE);

	    gps_current_buffer = gps_rx_bufferA;
	    gps_full_buffer = 0;
	    bActive = 0;
}


void gps_do()
{
	if(gps_got_line) /* process the data. */
	{
		//gps_rx_idx = 0;
		gps_got_line = 0;

		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);

		uint16_t bw;
		f_write(&gps_log, gps_full_buffer, 256, &bw);	/* Write data to the file */
		f_sync(&gps_log);

		//gps_full_buffer = 0;
	}

}

void gps_start()
{
	bActive = 1;

	USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);

	// Enable USCI_A0 RX interrupt
	USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

	f_mount(&FatFs, "", 0);		/* Give a work area to the default drive */
	f_open(&gps_log, "gpslog.txt", FA_WRITE | FA_CREATE_ALWAYS);

}

void gps_stop()
{
	bActive = 0;
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);

	USCI_A_UART_disableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // disable interrupt

	f_close(&gps_log);								/* Close the file */

	//f_mount(&FatFs, "", 0);		/* Give a work area to the default drive */
}


uint8_t gps_check()
{
	return bActive;
}



//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void USCI_A1_ISR(void)
{
	uint8_t c;

	 c = UCA1RXBUF;
	 GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
	 GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
	 gps_current_buffer[gps_rx_idx++] = c;

	 if(gps_rx_idx == 0) /* switch buffers */
	 {
		 GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN7);
		 gps_full_buffer = gps_current_buffer;
		 gps_got_line = 1;

		 if( gps_current_buffer == gps_rx_bufferA )
			 gps_current_buffer = gps_rx_bufferB;
		 else
			 gps_current_buffer = gps_rx_bufferA;
		 __bic_SR_register_on_exit(LPM0_bits);
	 }
}

