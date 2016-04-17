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
char kml_file_name[13] = "log000.kml";


const char kml_pre_data[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n\
<kml xmlns=\"http://earth.google.com/kml/2.1\">\r\n\
	<Document>\r\n\
		<name>17-02-2016_18-58-42</name>\r\n\
		<visibility>1</visibility>\r\n\
		<Folder>\r\n\
			<name>Track</name>\r\n\
			<Placemark>\r\n\
				<name>Track information</name>\r\n\
				<Style>\r\n\
					<LineStyle>\r\n\
						<color>C80000FF</color>\r\n\
						<width>3</width>\r\n\
					</LineStyle>\r\n\
				</Style>\r\n\
				<LineString>\r\n\
					<tessellate>1</tessellate>\r\n\
					<coordinates>\r\n";
const char kml_post_data[] ="\r\n"
		"</coordinates>\r\n\
				</LineString>\r\n\
				<description>\r\n\
					<![CDATA[Total time 00:34:34<br/>Distance 2.231Km<br/>Speed 3.873Km/h<br/>]]>\r\n\
				</description>\r\n\
			</Placemark>\r\n\
		</Folder>\r\n\
	</Document>\r\n\
</kml>\r\n";

uint8_t gps_new_data;
uint8_t gps_pos_lat[12];
uint8_t gps_pos_lon[12];
uint8_t gps_pos_string[64];
uint8_t *gps_pos_ptr;


//TinyGPS gps;


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
				if(fix)
					gps_pos_lat[0] = '0';
				gps_pos_ptr = &gps_pos_lat[1];
			}
			break;

	case 8:				// lat,
			if(c == ',')
			{
				state = 9;
				if( fix )
				{
					*gps_pos_ptr++ = '\0';
				}
			}
			else
				if( fix )
					*gps_pos_ptr++ = c;
			break;

	case 9:				// N,
			if(c == ',')
			{
				state = 10;
				if(fix)
					gps_pos_lon[0] = '0';
				gps_pos_ptr = &gps_pos_lon[1];
			}
			else if( c == 'S' && fix)
			{
				gps_pos_lat[0] = '-';
			}
			break;

	case 10:				// long,
			if(c == ',')
			{
				state = 11;
				if( fix )
				{
					*gps_pos_ptr++ = '\0';
				}

			}
			else
				if( fix )
					*gps_pos_ptr++ = c;
			break;

	case 11:				// W,
			if(c == ',')
				state = 12;
			else if( c == 'W' && fix )
			{
				gps_pos_lon[0] = '-';
			}
			break;

	case 12:				// Fix value
			if(c >= '0' && c <= '8')
			{
				fix = c - '0';
				gps_new_data = 1;
			}
			else
				fix = 0;
			state = 1;
			break;
	}
}

void re_format()
{
	/* no erro checking here, we assume we've got good data */
	/* in format DDMM.MMMM */
	/* Degrees, Minutes */

	/* output format */
	/* DD.DDDDDDD */
	/* Degrees */

	/* 1. extract minutes from ascii */

	/* 2. Divide by 60, produces value 0-1*/

	_iq24 lat = _atoIQ24(&gps_pos_lat[3]);

	lat = (lat / 60);

	_IQ24toa(&gps_pos_lat[3], "%0.7f", lat);

	_iq24 lon = _atoIQ24(&gps_pos_lon[4]);

	lon = (lon / 60);

	_IQ24toa(&gps_pos_lon[4], "%0.7f", lon);



	/* place back into string form */
	memcpy(gps_pos_string, gps_pos_lon, 11 );
	gps_pos_string[11] = ',';
	memcpy(&gps_pos_string[12], gps_pos_lat, 10 );
	memcpy(&gps_pos_string[22], ",0\n\0", 4 );


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

			if( gps_new_data )
			{
				gps_new_data = 0;
				FRESULT rc = f_open(&gps_log, kml_file_name, FA_WRITE | FA_OPEN_ALWAYS);
				if( rc )
					hal_led_b(RED);
				else
				{
					uint16_t bw;
					rc = f_lseek(&gps_log, gps_log.fsize);
					//rc |= f_write(&gps_log, gps_pos_string, strlen(gps_pos_string), (unsigned int*)&bw);
					re_format();
					rc |= f_write(&gps_log, gps_pos_string, strlen(gps_pos_string), (unsigned int*)&bw);
					if( rc || bw !=  strlen(gps_pos_string))
						hal_led_b(RED);
					f_close(&gps_log);
				}
			}
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
		hal_led_a(0);

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

	kml_file_name[5] = nmea_file_name[5];
	kml_file_name[4] = nmea_file_name[4];

	uint16_t bw;

	rc |= f_open(&gps_log, kml_file_name, FA_WRITE | FA_CREATE_ALWAYS);
	rc = f_write(&gps_log, (const void*)kml_pre_data, sizeof(kml_pre_data)-1, (unsigned int*)&bw);	/* Write data to the file */
	rc = f_close(&gps_log);
	/* something bad? no SD present? */
	if( rc != FR_OK )
	{
		hal_led_a(RED);
		hal_led_b(RED);
		gps_stop();
		return;
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

	f_open(&gps_log, kml_file_name, FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&gps_log, gps_log.fsize);
	f_write(&gps_log, (const void*)kml_post_data, sizeof(kml_post_data)-1, (unsigned int*)&bw);	/* Write data to the file */

	f_close(&gps_log);								/* Close the file */

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

