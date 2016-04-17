/*
 * gps.c
 *
 *  Created on: 29 Dec 2015
 *      Author: Gregory
 */

#include <msp430.h>
#include <stdint.h>
#include "IQmathLib.h"
#include "driverlib.h"

#include "USB_app/FatFs/ff.h"

#include "gps.h"
#include "hal.h"





volatile uint8_t* gps_current_buffer;
volatile uint8_t* gps_full_buffer;
volatile uint8_t gps_rx_bufferA[256];
volatile uint8_t gps_rx_bufferB[256];
volatile uint8_t gps_rx_idx;
volatile uint8_t gps_got_line;


char nmea_file_name[13] = "log000.txt";






FATFS FatFs;		/* FatFs work area needed for each volume */
FIL gps_log;			/* File object needed for each open file */



uint8_t bActive;
/* Use UCA1 */
void GPS_init()
{
	 //Configure UART pins (UCA1TXD/UCA1SIMO, UCA1RXD/UCA1SOMI)
	    //Set P4.4 and P4.5 as Module Function Input
	    GPIO_setAsPeripheralModuleFunctionInputPin(
	        GPIO_PORT_P3,
	        GPIO_PIN4 + GPIO_PIN3
	        );

	    P3REN &= ~(BIT4 | BIT3);

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

	    if(STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &param))
	    {
	        return; /* todo handle error? */
	    }

	    USCI_A_UART_enable(USCI_A0_BASE);

	    gps_current_buffer = gps_rx_bufferA;
	    gps_full_buffer = 0;
	    bActive = 0;


}

void GPS_deinit()
{
	 // set both as pull down to remove possiblity of bus contention.

	P3SEL &= ~(BIT3 | BIT4);
	GPIO_setAsInputPinWithPullDownResistor(
	        GPIO_PORT_P3,
	        GPIO_PIN4 + GPIO_PIN3
	        );


	    USCI_A_UART_disable(USCI_A0_BASE);

}

uint8_t fix = 0;
/* This is a simple, but long, state machine to determine if a valid GPS stream has a fix.
 * it is called one character at a time */
void gps_fix(uint8_t c)
{
	static uint8_t state = 0;
	switch( state )
	{
	case 0:
		if(c == '$')
			state = 1;
		break;
	case 1:
			if(c == 'G')
				state = 2;
			else
				state = 1;
			break;
	case 2:
			if(c == 'P')
				state = 3;
			else
				state = 1;
			break;

	case 3:
			if(c == 'G')
				state = 4;
			else
				state = 1;
			break;

	case 4:
			if(c == 'G')
				state = 5;
			else
				state = 1;
			break;

	case 5:
			if(c == 'A')
				state = 6;
			else
				state = 1;
			break;

	case 6:				// $GPGGA,
			if(c == ',')
				state = 7;
			else
				state = 1;
			break;

	case 7:				// time,
			if(c == ',')
			{
				state = 8;
			}
			break;

	case 8:				// lat,
			if(c == ',')
			{
				state = 9;
			}
			break;

	case 9:				// N,
			if(c == ',')
			{
				state = 10;
			}

			break;

	case 10:				// long,
			if(c == ',')
			{
				state = 11;
			}
			break;

	case 11:				// W,
			if(c == ',')
				state = 12;

			break;

	case 12:				// Fix value
			if(c >= '0' && c <= '8')
			{
				fix = c - '0';
			}
			else
				fix = 0;
			state = 1;
			break;
	}
}


void gps_do()
{
	//re_format();
	if(gps_got_line) /* process the data. */
	{

		uint16_t bytes_in_buffer = sizeof(gps_rx_bufferA);
		uint8_t *c = (uint8_t*)gps_full_buffer;
		do{
			gps_fix(*c++);
			if( fix == 1 || fix == 2 )
				hal_led_b(GREEN);
			else
				hal_led_b(RED);
		} while(--bytes_in_buffer);

		gps_got_line = 0;


		FRESULT rc;
		uint16_t bw;
		rc = f_open(&gps_log, nmea_file_name, FA_WRITE | FA_OPEN_ALWAYS);
		if( rc )
			hal_led_a(YELLOW);

		rc = f_lseek(&gps_log, gps_log.fsize);
		if( rc )
			hal_led_a(YELLOW);

		rc = f_write(&gps_log, (const void*)gps_full_buffer, 256, (unsigned int*)&bw);	/* Write data to the file */
		if( rc )
			hal_led_a(YELLOW);

		rc = f_close(&gps_log);
		if( rc )
			hal_led_a(YELLOW);


		//gps_full_buffer = 0;
	}

}

void gps_start()
{
	bActive = 1;


	f_mount(&FatFs, "", 0);		/* Give a work area to the default drive */



	FRESULT rc;
	/* determine the next log file number. */
	for( uint8_t file_number = 0 ; file_number < 99 ; file_number++ )
	{
		nmea_file_name[5] = file_number % 10 + '0';
		nmea_file_name[4] = file_number /10 % 10 + '0';


		// this call will fail if the file does exist.
		rc = f_open(&gps_log, nmea_file_name, FA_WRITE | FA_CREATE_NEW);

		// file opened sucessfully? we are done
		if( rc == FR_OK )
			break;


	}


	hal_led_b(YELLOW);

	hal_gps_pwr_on();

	USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);

		// Enable USCI_A0 RX interrupt
		USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt




}

void gps_stop()
{
	bActive = 0;

	USCI_A_UART_disableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // disable interrupt

	uint16_t bw;


	hal_led_b(0);
	/* unmount work area */
	f_mount(0, "", 0);		/* Give a work area to the default drive */
	fix = 0;
	hal_gps_pwr_off();
}


uint8_t gps_check()
{
	return gps_got_line;
}



//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************

__attribute__((interrupt(USCI_A0_VECTOR)))

void USCI_A0_ISR(void)
{
	uint8_t c;



	 c = UCA0RXBUF;
	 gps_current_buffer[gps_rx_idx++] = c;

	 if(gps_rx_idx == 0) /* switch buffers */
	 {
		 gps_full_buffer = gps_current_buffer;
		 gps_got_line = 1;

		 if( gps_current_buffer == gps_rx_bufferA )
			 gps_current_buffer = gps_rx_bufferB;
		 else
			 gps_current_buffer = gps_rx_bufferA;
		 __bic_SR_register_on_exit(LPM3_bits);
	 }

	 /* visual indication of data */
	 if( c == '$')
		 hal_led_a(GREEN);
	 if( c == '\n')
		 hal_led_a(0);
}

