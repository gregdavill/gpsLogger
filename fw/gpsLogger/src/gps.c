/*
 * gps.c
 *
 *  Created on: 29 Dec 2015
 *      Author: Gregory
 */

#include <msp430.h>
#include "driverlib.h"
#include <stdint.h>
#include <stdlib.h>
#include "IQmathLib.h"

#include "USB_app/FatFs/ff.h"

#include "gps.h"
#include "hal.h"

volatile uint8_t* fifo_data = 0x1c00;
volatile uint16_t fifo_head;
volatile uint16_t fifo_tail;
// volatile uint8_t* fifo_data;
const uint16_t fifo_size
	= 2048; // stare incoming USART buffer in USB ram. This gives us a large buffer without wasting RAM.

char nmea_file_name[] = "nmea/log000.txt";

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
const char NMEA_header[]
	= "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\r\n"
	  "     gpsLogger [https://github.com/Greeeg/gpsLogger]\r\n"
	  "\r\n"
	  "  Version: " TOSTRING(_VCS_COMMIT_) "\r\n"
										   "  Build: " __DATE__ " " __TIME__
										   "\r\n=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\r\n";

const char xml_a[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<kml "
					 "xmlns=\"http://earth.google.com/kml/2.1\">\r\n\t<Document>\r\n\t<name>"; // 17-02-2016_18-58-42
const char xml_b[] = "</name>\r\n\t\t<visibility>1</visibility>\r\n\t\t<Folder>\r\n\t\t\t<name>Track";
const char xml_c[] = "</name>\r\n\t\t\t<Placemark>\r\n\t\t\t\t<name>Track information";
const char xml_d[] = "</name>\r\n\t\t\t\t<Style>\r\n\t\t\t\t\t<LineStyle>\r\n\t\t\t\t\t\t<color>ff03ffff";
const char xml_e[] = "</color>\r\n\t\t\t\t\t\t<width>3</width>\r\n\t\t\t\t\t</LineStyle>\r\n\t\t\t\t</"
					 "Style>\r\n\t\t\t\t<LineString>\r\n\t\t\t\t\t<tessellate>1</"
					 "tessellate>\r\n\t\t\t\t\t<coordinates>\r\n";

const char xml_f[] = "\t\t\t\t\t</coordinates>\r\n\t\t\t\t</LineString>\r\n\t\t\t\t<description>\r\n\t\t\t\t</"
					 "description>\r\n\t\t\t</Placemark>\r\n\t\t</Folder>\r\n\t</Document>\r\n</kml>\r\n";

FATFS FatFs;   /* FatFs work area needed for each volume */
FIL   gps_log; /* File object needed for each open file */
FIL   kml_file;

uint8_t fifo_available()
{
	uint8_t return_value;

	__bic_SR_register(GIE);
	return_value = (fifo_head != fifo_tail);
	__bis_SR_register(GIE);

	return return_value;
}

void fifo_push(uint8_t _data)
{
	//_disable_interrupt();

	uint16_t next = fifo_head + 1;
	if (next >= fifo_size) {
		next = 0;
	}
	fifo_data[next] = _data;

	fifo_head = next;

	if (next == fifo_tail) /* Overflow? */
	{
		fifo_init();
		if (++fifo_tail > fifo_size) {
			fifo_tail = 0;
		}
	}

	//	_enable_interrupt();
}

uint8_t fifo_pop()
{
	if (!fifo_available())
		return 0;

	uint8_t _data;
	__bic_SR_register(GIE);

	uint16_t next = fifo_tail + 1;
	if (next >= fifo_size) {
		next = 0;
	}
	_data = fifo_data[next];

	fifo_tail = next;

	__bis_SR_register(GIE);
	return _data;
}

void fifo_init()
{
	fifo_head = 0;
	fifo_tail = 0;
}

uint8_t bActive, b_KmlFileOpen;
/* Use UCA1 */
void GPS_init()
{
	// Configure UART pins (UCA1TXD/UCA1SIMO, UCA1RXD/UCA1SOMI)
	// Set P4.4 and P4.5 as Module Function Input
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN4 + GPIO_PIN3);

	P3REN &= ~(BIT4 | BIT3);

	// Baudrate = 9600, clock freq = 1.048MHz
	// UCBRx = 109, UCBRFx = 0, UCBRSx = 2, UCOS16 = 0
	USCI_A_UART_initParam param = { 0 };
	param.selectClockSource		= USCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar		= 866;
	param.firstModReg			= 0;
	param.secondModReg			= 2;
	param.parity				= USCI_A_UART_NO_PARITY;
	param.msborLsbFirst			= USCI_A_UART_LSB_FIRST;
	param.numberofStopBits		= USCI_A_UART_ONE_STOP_BIT;
	param.uartMode				= USCI_A_UART_MODE;
	param.overSampling			= USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

	if (STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &param)) {
		return; /* todo handle error? */
	}

	USCI_A_UART_enable(USCI_A0_BASE);

	gps_util_init();
}

void gps_putc(uint8_t c)
{
	UCA0TXBUF = c;

	while (UCA0STAT & UCBUSY)
		;
}

void gps_puts(uint8_t* s)
{
	while (*s) {
		gps_putc(*s++);
	}
}

FRESULT file_sync(FIL* file)
{
	FRESULT rc;
	/* I've found that the SD card sometimes rejects a f_sync on the first try */
		uint8_t timeout = 32;
		while (--timeout) {
			rc = f_sync(file);
			if (rc == FR_OK)
				return FR_OK;
		}

		return rc;
}

/**
 \brief ubx_transmit_message - Append checksum and transmit a UBX message to a Ublox GPS.

 \param data - Header + Payload of message
 \param len - Length of message (Max 30 bytes)

 \return void
 */
void ubx_transmit_message(const char* data, int len)
{
	uint8_t message_buffer[64];

	/* Stop buffer overflows */
	if (len > 30) {
		return;
	}

	/* Copy message payload to buffer, make use of the DMA */
	memcpy(message_buffer, (void*)data, len);

	/*  Calculate checksum
	 Ublox has uses an 8bit fletcher Algorithm over the main payload of the messages.
	 See Programming manual: UBX-13003221
	 */
	uint8_t ck_a = 0, ck_b = 0;

	for (int i = 2; i < len; i++) {
		ck_a += message_buffer[i];
		ck_b += ck_a;
	}

	/* Append checksum to data and increment length to match */
	message_buffer[len++] = ck_a;
	message_buffer[len++] = ck_b;

	/* Write entire string to GPS */
	for (uint16_t i = 0; i < len; i++) {
		gps_putc(message_buffer[i]);
	}
}

void gps_ublox_send()
{
	const char change_talker[] = { 0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00,
								   0x00, 0x00, 0x01, 0x00, 0x01, 'G',  'P',  0,	0,	0,	0,	0,	0 };

	/* Change talker to "GP" */
	ubx_transmit_message(change_talker, sizeof(change_talker));

	/* Disable GLL */
	const char disable_gll[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00 };
	ubx_transmit_message(disable_gll, sizeof(disable_gll));

	/* Slow GSV */
	const char disable_gsv[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x10 };
	ubx_transmit_message(disable_gsv, sizeof(disable_gsv));

	/* Slow GSA */
	const char disable_gsa[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x10 };
	ubx_transmit_message(disable_gsa, sizeof(disable_gsa));

	/* Disable VTG */
	const char disable_vtg[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00 };
	ubx_transmit_message(disable_vtg, sizeof(disable_vtg));

	uint16_t transmission_speed = 500;
	/* Boost Transmit rate to 2Hz */
	const char boost_rate[]
		= { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (char)(transmission_speed & 0xFF), (char)(transmission_speed >> 8),
			1,	0,	0,	0 };
	ubx_transmit_message(boost_rate, sizeof(boost_rate));

	/* Delay removes all those ublox binary ack messages which could mees up our NEMA output file. */
	delay_ms(200);
}

void GPS_deinit()
{
	// set both as pull down to remove possiblity of bus contention.

	P3SEL &= ~(BIT3 | BIT4);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN4 + GPIO_PIN3);

	USCI_A_UART_disable(USCI_A0_BASE);
}

uint8_t file_date[10];
uint8_t file_time[10];
uint8_t gps_time_invalid = 0;
uint8_t init_gps;

void gps_do()
{
	if (fifo_available()) /* process the data. */
	{
		if (init_gps == 1) {
			hal_led_a(CYAN);

			gps_puts("$PMTK300,500,0,0,0,0*28\r\n");
			gps_puts("$PMTK220,500*2B\r\n");

			/* Ublox sentences */
			gps_ublox_send();

			init_gps = 0;
			// return;
		}

		while (fifo_available()) {
			if (gps_util_valid(fifo_pop())) /* returns true when a correct checksum is processed */
			{
				char* gps_line = gps_util_get_last_valid_line();

				if (!gps_line)
					continue;

				{
					FRESULT  rc;
					uint16_t bw, len;
					len				= strlen(gps_line);
					gps_line[len++] = '\r';
					gps_line[len++] = '\n';
					gps_line[len]   = '\0';

					rc = f_write(&gps_log, (const void*)gps_line, len, (unsigned int*)&bw); /* Write data to the file */
										if (rc || len != bw) {
											hal_led_a(YELLOW);
										}

										rc = file_sync(&gps_log);
															if (rc) {
																hal_led_a(YELLOW);
															}
				}

				if (gps_util_is_PMTK(gps_line)) {
					hal_compatibility_set();
				}

				if (gps_util_is_RMC(gps_line)) {
					/* visual indication of data */
					static uint8_t _flipper = 0;
					if (_flipper ^= 1) {
						hal_led_a(GREEN);
					} else
						hal_led_a(0);

					if (gps_util_fix_valid(gps_line)) {
						hal_led_b(GREEN);
						// if (gps_time_invalid) {
						/* Ceck if we have a valid date in string */
						if (gps_util_extract_date(gps_line, file_date) != 0
							&& gps_util_extract_time(gps_line, file_time) != 0) {
							gps_util_update_timezone(file_date, file_time);
							gps_time_invalid = 0;

						} else {
							gps_time_invalid = 1;
						}

						if(gps_time_invalid == 0)
						{
							/* If File open write our data to it. */
							if (b_KmlFileOpen) {
								gps_convert_NMEA2coords(gps_line);
							} else {
								/* Otherwise try to create file */
								gps_create_kml_file();
							}
						}

					} else {
						hal_led_b(RED);
					}
				}
			}
		}
	}
}

void gps_start()
{
	bActive		  = 1;
	b_KmlFileOpen = 0;

	f_mount(&FatFs, "", 1); /* Give a work area to the default drive */

	FRESULT rc;
	/* we want to store logs into nma folder, ensure it exists */
	rc = f_mkdir("NMEA");
	if ((rc != FR_EXIST) && (rc != FR_OK))
		hal_led_a(RED);

	/* determine the next log file number. */
	for (uint16_t file_number = 0; file_number < 999; file_number++) {
		nmea_file_name[10] = file_number % 10 + '0';
		nmea_file_name[9]  = file_number / 10 % 10 + '0';
		nmea_file_name[8]  = file_number / 100 % 10 + '0';

		// this call will fail if the file does exist.
		rc = f_open(&gps_log, nmea_file_name, FA_WRITE | FA_CREATE_NEW);

		// file opened sucessfully? we are done
		if (rc == FR_OK)
			break;
	}

	UINT bw = 0;
	rc		= f_write(&gps_log, NMEA_header, sizeof(NMEA_header) - 1, &bw);
	if (rc)
		hal_led_a(RED);


	rc = file_sync(&gps_log);
	if (rc) {
		hal_led_a(RED);
	}

	hal_led_b(RED);

	gps_time_invalid = 1;
	init_gps		 = 1;

	fifo_init();
	hal_gps_pwr_on(); /* calls gps_init() */

	USCI_A_UART_clearInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
	USCI_A_UART_enableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt
}

void check_new_firmware()
{
	f_mount(&FatFs, "", 0); /* Give a work area to the default drive */

	FRESULT rc;

	// this call will fail if the file does exist.
	rc = f_open(&gps_log, "firmware.bin", FA_READ);

	if (rc == FR_OK) {
		WDTCTL = WDT_ARST_16;

		while (1)
			;
	}
}

void gps_minute2degree(uint8_t* minute_string, uint8_t* output)
{
	_iq24 minutes = _atoIQ24(minute_string);
	minutes /= 60;
	_IQ24toa(output, "%0.7f", minutes);
}

extern uint8_t RWbuf[512]; // make use of USB memory when USB isn't in use

void gps_convert_NMEA2coords1(char* gps_line)
{
	uint16_t bw;

	FRESULT rc = f_write(&kml_file, gps_line, strlen(gps_line), &bw);
	if (rc) {
		hal_led_a(RED);
	}

	rc = file_sync(&kml_file);
	if (rc) {
		hal_led_a(RED);
	}
}

void gps_convert_NMEA2coords(char* gps_line)
{
	/* recycle USB RAM for more buffer space */
	uint8_t* degree_string	= &RWbuf[128];
	uint8_t* minute_string	= &RWbuf[140];
	uint8_t* degrees_fraction = &RWbuf[160];
	uint8_t* _buff			  = &RWbuf[200];
	uint8_t  negate;

	memset(&RWbuf[128], 0, 256);

	gps_util_extract_long_degrees(gps_line, degree_string);
	gps_util_extract_long_minutes(gps_line, minute_string);
	gps_minute2degree(minute_string, degrees_fraction);

	negate = gps_util_extract_west(gps_line);

	/* format back into a ascii string */
	uint8_t* buff_ptr = _buff;

	strcpy(buff_ptr, "\t\t\t\t\t");
	buff_ptr += strlen("\t\t\t\t\t");

	if (negate)
		*buff_ptr++ = '-';

	strcpy(buff_ptr, degree_string);
	buff_ptr += strlen(degree_string);

	strcpy(buff_ptr, degrees_fraction);
	buff_ptr += strlen(degrees_fraction);

	*buff_ptr++ = ',';

	// repeat for lon
	gps_util_extract_lat_degrees(gps_line, degree_string);
	gps_util_extract_lat_minutes(gps_line, minute_string);
	gps_minute2degree(minute_string, degrees_fraction);

	negate = gps_util_extract_north(gps_line);

	if (negate)
		*buff_ptr++ = '-';

	strcpy(buff_ptr, degree_string);
	buff_ptr += strlen(degree_string);

	strcpy(buff_ptr, degrees_fraction);
	buff_ptr += strlen(degrees_fraction);

	strcpy(buff_ptr, ",0.0\r\n");

	uint16_t bw;

	FRESULT rc = f_write(&kml_file, _buff, strlen(_buff), &bw);
	if (rc) {
		hal_led_a(RED);
	}

	rc = file_sync(&kml_file);
	if (rc) {
		hal_led_a(YELLOW);
	}
}

extern uint16_t gps_year;
extern uint8_t  gps_day, gps_month;
extern uint8_t  gps_hour, gps_minute, gps_second;

void create_formated_time(char* in, char* out)
{
	/* convert 072323 into 0723h */
	out[0] = '0' + (gps_hour / 10 % 10);
	out[1] = '0' + (gps_hour % 10);
	out[2] = '_';
	out[3] = '0' + (gps_minute / 10 % 10);
	out[4] = '0' + (gps_minute % 10);
	out[5] = 'h';
	out[6] = 0;
}

void create_formated_date(char* out)
{
	static const char const_months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	/* convert ddmmyy into dd mmm yy */
	out[0] = '0' + (gps_day / 10 % 10);
	out[1] = '0' + (gps_day % 10);
	out[2] = ' ';

	// memcpy(&out[3], &const_months[(gps_month - 1) * 3], 3);
	/* unrolled memcpy */
	char* ptr = (char*)&const_months[(gps_month - 1) * 3];
	out[3]	= *ptr++;
	out[4]	= *ptr++;
	out[5]	= *ptr;

	out[6] = ' ';

	//_IQ2toa(&out[7], "%04.00f", _IQ2(gps_year));
	out[7]  = '0' + (gps_year / 1000 % 10);
	out[8]  = '0' + (gps_year / 100 % 10);
	out[9]  = '0' + (gps_year / 10 % 10);
	out[10] = '0' + (gps_year % 10);
	out[11] = 0;
}

void create_filename_date(char* out)
{
	out[0] = '0' + (gps_year / 1000 % 10);
	out[1] = '0' + (gps_year / 100 % 10);
	out[2] = '0' + (gps_year / 10 % 10);
	out[3] = '0' + (gps_year % 10);

	out[4] = '0' + (gps_month / 10 % 10);
	out[5] = '0' + (gps_month % 10);

	out[6] = '0' + (gps_day / 10 % 10);
	out[7] = '0' + (gps_day % 10);
	out[8] = '-';

	out[9]  = '0' + (gps_hour / 10 % 10);
	out[10] = '0' + (gps_hour % 10);

	out[11] = '0' + (gps_minute / 10 % 10);
	out[12] = '0' + (gps_minute % 10);

	out[13] = '0' + (gps_second / 10 % 10);
	out[14] = '0' + (gps_second % 10);
}

void gps_create_kml_file()
{
	// uint8_t short_filename[13] = "TRACK---.KML";
	uint8_t filename[26] = "YYYYMMDD-HHMMSS.KML";

	/* Copy log number into "---" area */
	// memcpy(&short_filename[5], &nmea_file_name[8], 3);
	create_filename_date(filename);

	uint16_t bw;
	FRESULT  rc = f_open(&kml_file, filename, FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		/* If error creating file, early exit. It will try again next time */
		hal_led_a(YELLOW);
		return;
	}
	rc |= f_write(&kml_file, xml_a, sizeof(xml_a) - 1, &bw);
	if (rc)
		hal_led_a(YELLOW);

	// name
	rc |= f_write(&kml_file, filename, strlen(filename), &bw);
	if (rc)
		hal_led_a(YELLOW);

	rc |= f_write(&kml_file, xml_b, sizeof(xml_b) - 1, &bw);
	if (rc)
		hal_led_a(YELLOW);

	// track

	rc |= f_write(&kml_file, xml_c, sizeof(xml_c) - 1, &bw);
	if (rc)
		hal_led_a(YELLOW);

	// info

	rc |= f_write(&kml_file, xml_d, sizeof(xml_d) - 1, &bw);
	if (rc)
		hal_led_a(YELLOW);

	rc |= f_write(&kml_file, xml_e, sizeof(xml_e) - 1, &bw);
	if (rc)
		hal_led_a(YELLOW);

	rc |= f_sync(&kml_file);
	if (rc)
		hal_led_a(YELLOW);

	if (rc == FR_OK)
		b_KmlFileOpen = 1;
}

void gps_stop()
{
	USCI_A_UART_disableInterrupt(USCI_A0_BASE, USCI_A_UART_RECEIVE_INTERRUPT); // disable interrupt

	uint16_t bw;
	FRESULT  rc;

	hal_gps_pwr_off();

	if (!bActive)
		return;

	bActive = 0;

	rc = f_write(&gps_log, "\r\n=-=-=-=-=-=-=-=-=-=-=\r\nClean Power off\r\n=-=-=-=-=-=-=-=-=-=-=\r\n", 65, &bw);
	if (rc)
		hal_led_a(YELLOW);

	rc = f_close(&gps_log);
	if (rc)
		hal_led_a(YELLOW);

	/* Do we need to close the KML file? Is it open? */

	if (b_KmlFileOpen) {
		rc = f_write(&kml_file, xml_f, sizeof(xml_f) - 1, &bw);
		if (rc)
			hal_led_a(YELLOW);

		rc = f_close(&kml_file);
		if (rc)
			hal_led_a(YELLOW);
	}

	hal_led_b(0);
	/* unmount work area */
	f_mount(0, "", 1); /* Give a work area to the default drive */
}

__attribute__((interrupt(USCI_A0_VECTOR))) void USCI_A0_ISR(void)
{
	fifo_push(UCA0RXBUF);
	__bic_SR_register_on_exit(LPM0_bits);
}
