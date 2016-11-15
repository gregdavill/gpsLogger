/*
 * gps.c
 *
 *  Created on: 29 Dec 2015
 *      Author: Gregory
 */

#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>
#include "IQmathLib.h"
#include "driverlib.h"

#include "USB_app/FatFs/ff.h"

#include "gps.h"
#include "hal.h"


#define GPS_BUFFER_SIZE 32
#define GPS_BUFFERS 8


volatile uint8_t* gps_current_buffer;
volatile uint8_t* gps_full_buffer;
volatile uint8_t gps_rx_buffer[GPS_BUFFER_SIZE * GPS_BUFFERS];
volatile uint16_t gps_rx_idx;
volatile uint8_t gps_buffer_idx;
volatile uint8_t gps_got_line;

char nmea_file_name[] = "nmea/log000.txt";

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
const char NMEA_header[] = "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\r\n     gpsLogger [https://github.com/Greeeg/gpsLogger]\r\n\r\n  Version: " TOSTRING(_VCS_COMMIT_) "\r\n  Build: " __DATE__ " " __TIME__ "\r\n=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\r\n";

const char xml_a[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\r\n\t<Document>\r\n\t<name>";//17-02-2016_18-58-42
const char xml_b[] = "</name>\r\n\t\t<visibility>1</visibility>\r\n\t\t<Folder>\r\n\t\t\t<name>Track";
const char xml_c[] = "</name>\r\n\t\t\t<Placemark>\r\n\t\t\t\t<name>Track information";
const char xml_d[] = "</name>\r\n\t\t\t\t<Style>\r\n\t\t\t\t\t<LineStyle>\r\n\t\t\t\t\t\t<color>ff03ffff";
const char xml_e[] = "</color>\r\n\t\t\t\t\t\t<width>3</width>\r\n\t\t\t\t\t</LineStyle>\r\n\t\t\t\t</Style>\r\n\t\t\t\t<LineString>\r\n\t\t\t\t\t<tessellate>1</tessellate>\r\n\t\t\t\t\t<coordinates>\r\n";

const char xml_f[] = "\t\t\t\t\t</coordinates>\r\n\t\t\t\t</LineString>\r\n\t\t\t\t<description>\r\n\t\t\t\t</description>\r\n\t\t\t</Placemark>\r\n\t\t</Folder>\r\n\t</Document>\r\n</kml>\r\n";

FATFS FatFs;		/* FatFs work area needed for each volume */
FIL gps_log;			/* File object needed for each open file */
FIL kml_file;


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


	    gps_util_init();

	    gps_full_buffer = 0;
	    gps_rx_idx = 0;
	    bActive = 0;


}

void gps_putc(uint8_t c)
{
	UCA0TXBUF = c;

	while(UCA0STAT & UCBUSY);
}

void gps_puts(uint8_t* s)
{
	while(*s)
	{
		gps_putc(*s++);
	}
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

uint8_t file_date[10];
uint8_t file_time[10];
uint8_t gps_time_invalid = 0;
uint8_t init_gps;

void gps_do()
{
	//re_format();
	if(gps_got_line) /* process the data. */
	{


		if(init_gps)
		{
			gps_puts("$PMTK300,500,0,0,0,0*28\r\n");
			gps_puts("$PMTK220,500*2B\r\n");
			init_gps = 0;
			return;
		}

		gps_got_line = 0;
		uint8_t *c = (uint8_t*)gps_full_buffer;
		uint16_t idx = 0;
		for( ; idx < GPS_BUFFER_SIZE; ++idx)
		{

			if( gps_util_valid(*c++) ) /* returns true when a correct checksum is processed */
			{

				char* gps_line = gps_util_get_last_valid_line();

				if( !gps_line )
					break;

				if( gps_util_is_RMC(gps_line) )
				{
					/* visual indication of data */
					static uint8_t _flipper = 0;
					if( _flipper ^= 1 )
					{
						hal_led_a(GREEN);
					}
					else
						hal_led_a(0);

					if( gps_util_fix_valid(gps_line) )
					{
						hal_led_b(GREEN);
						if(gps_time_invalid)
						{
							gps_util_extract_date(gps_line, file_date);
							gps_util_extract_time(gps_line, file_time);

							gps_util_update_timezone(file_date, file_time);
							gps_time_invalid = 0;

							gps_create_kml_file(file_date, file_time);
						}

						gps_convert_NMEA2coords(gps_line);
					}
					else
					{
						hal_led_b(RED);
					}
				}
			}

		}



		FRESULT rc;
		uint16_t bw;
		rc = f_write(&gps_log, (const void*)gps_full_buffer, idx, (unsigned int*)&bw);	/* Write data to the file */
		if( rc )
		{
			hal_led_a(YELLOW);
		}
		rc = f_sync(&gps_log);
		if( rc )
		{
			hal_led_a(YELLOW);
		}


		//gps_full_buffer = 0;
	}

}

void gps_start()
{
	bActive = 1;


	f_mount(&FatFs, "", 0);		/* Give a work area to the default drive */

	FRESULT rc;
	/* we want to store logs into nma folder, ensure it exists */
	rc = f_mkdir("nmea");
    if (rc != FR_EXIST || rc != FR_OK)
    	hal_led_a(RED);

	/* determine the next log file number. */
	for( uint16_t file_number = 0 ; file_number < 999 ; file_number++ )
	{
		nmea_file_name[10] = file_number % 10 + '0';
		nmea_file_name[9] = file_number /10 % 10 + '0';
		nmea_file_name[8] = file_number /100 % 10 + '0';


		// this call will fail if the file does exist.
		rc = f_open(&gps_log, nmea_file_name, FA_WRITE | FA_CREATE_NEW);

		// file opened sucessfully? we are done
		if( rc == FR_OK )
			break;


	}

	UINT bw = 0;
	rc = f_write(&gps_log, NMEA_header, sizeof(NMEA_header) - 1, &bw );
	if( rc )
		hal_led_a(RED);

	hal_led_b(RED);

	hal_gps_pwr_on(); /* calls gps_init() */

	gps_current_buffer = gps_rx_buffer;
	gps_buffer_idx = 0;
	gps_rx_idx = 0;
	gps_got_line = 0; // clear flags
	gps_time_invalid = 1;
	init_gps = 1;

	USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);

	// Enable USCI_A0 RX interrupt
	USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

}


void gps_minute2degree(uint8_t* minute_string, uint8_t* output)
{
	_iq24 minutes = _atoIQ24(minute_string);
	minutes /= 60;
	_IQ24toa(output, "%0.7f", minutes);
}



extern uint8_t RWbuf[512]; // make use of USB memory when USB isn't in use

void gps_convert_NMEA2coords(char* gps_line)
{

	/* recycle USB RAM for more buffer space */
	uint8_t* degree_string = RWbuf;
	uint8_t* minute_string = &RWbuf[4];
	uint8_t* degrees_fraction = &RWbuf[12];
	uint8_t* _buff = &RWbuf[24];
	uint8_t negate;


	memset(RWbuf,0,128);

	gps_util_extract_long_degrees(gps_line, degree_string);
	gps_util_extract_long_minutes(gps_line, minute_string);
	gps_minute2degree(minute_string, degrees_fraction);

	negate = gps_util_extract_west(gps_line);

	/* format back into a ascii string */
	uint8_t* buff_ptr = _buff;

	strcpy(buff_ptr, "\t\t\t\t\t");
	buff_ptr += strlen("\t\t\t\t\t");



	if(negate)
		*buff_ptr++ = '-';

	strcpy(buff_ptr, degree_string);
	buff_ptr += strlen(degree_string);

	strcpy(buff_ptr, degrees_fraction);
	buff_ptr += strlen(degrees_fraction);

	*buff_ptr++ = ',';

	// repeat for long
	gps_util_extract_lat_degrees(gps_line, degree_string);
	gps_util_extract_lat_minutes(gps_line, minute_string);
	gps_minute2degree(minute_string, degrees_fraction);

	negate = gps_util_extract_north(gps_line);

	if(negate)
		*buff_ptr++ = '-';

	strcpy(buff_ptr, degree_string);
	buff_ptr += strlen(degree_string);

	strcpy(buff_ptr, degrees_fraction);
	buff_ptr += strlen(degrees_fraction);

	strcpy(buff_ptr, ",0.0\r\n");

	uint16_t bw;

	FRESULT rc = f_write(&kml_file, _buff, strlen(_buff), &bw);
	if( rc )
	{
		hal_led_a(RED);
	}

	rc = f_sync(&kml_file);
	if( rc )
	{
		hal_led_a(RED);
	}

}


extern uint16_t gps_year;
extern uint8_t gps_day,gps_month;
extern uint8_t gps_hour,gps_minute,gps_second;


void pretty_time(char* in, char* out)
{
	/* convert 072323 into 0723hours */
	_IQ2toa(out, "%02.00f", _IQ2(gps_hour));
	out[2] = '_';
	_IQ2toa(&out[3], "%02.00f", _IQ2(gps_minute));
	out[5] = 'h';
	out[6] = 0;
}

void pretty_date(char* in, char* out)
{
	static const char const_months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	/* convert ddmmyy into dd mmm yy */
	_IQ2toa(out, "%02.00f", _IQ2(gps_day));
	out[2] = ' ';

	memcpy(&out[3], &const_months[(gps_month - 1)*3], 3);

	out[6] = ' ';

	_IQ2toa(&out[7], "%04.00f", _IQ2(gps_year));
	out[11] = 0;
}

void gps_create_kml_file(char* date, char* time)
{

	uint8_t time_string[15];
	uint8_t date_string[15];
	uint8_t long_filename[32];
	uint8_t *ptr = long_filename;

	pretty_date(date, date_string);
	pretty_time(time, time_string);

	strcpy(ptr, date_string);
	ptr = long_filename + strlen(long_filename);
	strcpy(ptr, " ");
	ptr = long_filename + strlen(long_filename);
	strcpy(ptr, time_string);
	ptr = long_filename + strlen(long_filename);
	strcpy(ptr, ".kml");

	uint16_t bw;
	FRESULT rc = f_open(&kml_file, long_filename, FA_WRITE | FA_CREATE_ALWAYS);
	if( rc )
	{
		// attempt to append _1 to file
		strcpy(ptr, "_1.kml");
		rc = f_open(&kml_file, long_filename, FA_WRITE | FA_CREATE_ALWAYS);

		hal_led_a(YELLOW);
	}
	rc = f_write(&kml_file, xml_a, sizeof(xml_a) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);

	// name
	rc = f_write(&kml_file, long_filename, strlen(long_filename), &bw);
	if( rc )
			hal_led_a(YELLOW);


	rc = f_write(&kml_file, xml_b, sizeof(xml_b) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);

	// track


	rc = f_write(&kml_file, xml_c, sizeof(xml_c) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);

	// info

	rc = f_write(&kml_file, xml_d, sizeof(xml_d) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);


	rc = f_write(&kml_file, xml_e, sizeof(xml_e) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);

	rc = f_sync(&kml_file);
	if( rc )
			hal_led_a(YELLOW);


}

void gps_stop()
{


	USCI_A_UART_disableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // disable interrupt

	uint16_t bw;
	FRESULT rc;

	hal_gps_pwr_off();

	if(!bActive)
			return;

	bActive = 0;


	rc = f_write(&gps_log, "=-=-=-=-=-=-=-=-=-=-=\r\nClean Power off\r\n=-=-=-=-=-=-=-=-=-=-=\r\n", 63, &bw);
	if( rc )
			hal_led_a(YELLOW);


	rc = f_close(&gps_log);
	if( rc )
			hal_led_a(YELLOW);

	rc = f_write(&kml_file, xml_f, sizeof(xml_f) - 1, &bw);
	if( rc )
			hal_led_a(YELLOW);

	rc = f_close(&kml_file);
	if( rc )
			hal_led_a(YELLOW);


	hal_led_b(0);
	/* unmount work area */
	f_mount(0, "", 0);		/* Give a work area to the default drive */


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

	 if(gps_rx_idx >= GPS_BUFFER_SIZE) /* switch buffers */
	 {
		 gps_rx_idx = 0;
		 gps_full_buffer = gps_current_buffer;
		 gps_got_line = 1;

		 /* get next buffer */
		 if( ++gps_buffer_idx >= (GPS_BUFFERS))
			 gps_buffer_idx = 0;

		 gps_current_buffer = &gps_rx_buffer[gps_buffer_idx*GPS_BUFFER_SIZE];

		 __bic_SR_register_on_exit(LPM0_bits);
	 }
}

