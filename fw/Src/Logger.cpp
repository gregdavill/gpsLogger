/*
 * Logger.cpp
 *
 *  Created on: 12 Jul 2016
 *      Author: greg
 */

#include "mark3.h"

#include <fabooh.h>

#include "ff.h"
#include "dvrMMC.h"

#include "IQmathLib.h"

#include "gps.h"

EventFlag m_event;
//MailBox mb;

void LoggerError(FRESULT f_res)
{
	LED_A_RED::low();
	LED_B_RED::high();
	LED_B_GREEN::high();
	LED_B_BLUE::high();

	while(1)
	{
		/* FLASH LEDS */
		for(uint8_t idx = 0 ; idx < f_res; idx++)
		{
			LED_A_RED::toggle();
			LED_B_RED::toggle();

			Thread::Sleep(100);
		}

		Thread::Sleep(500);
	}
}


void LoggerLedCallback(Thread *_owner, void *_data)
{
	/* Turn off LED */
	LED_A_RED::high();
	LED_A_BLUE::high();
	LED_A_GREEN::high();
}

static void rxCall(msp430GPS* _gps)
{
	uint8_t d = 1;//_gps->Read(0,0);
	m_event.Set(d);
}

void LoggerMain(void *_q)
{

	FATFS fatfs;
	FIL logger_file;

	FRESULT f_res;

	unsigned int bytes_written;

	char nmea_file_name[] = "log000.txt";

	/* Power up SD card and access disk */
	MMC_POWER_ENABLE::high();
	Thread::Sleep(50);

	m_event.Init();

	/* Get GPS driver */
	Driver *gps = DriverList::FindByPath("/dev/gps");

	/* Get MMC driver */
	Driver *mmc = DriverList::FindByPath("/dev/mmc");

	gps->Control(CMD_SET_RX_CALLBACK, (void*)rxCall, 0,0,0);

	/* Note will allocate memory here */
	gps->Open();

	/* These should probably be included in the GPS driver... */
	GPS_POWER_ENABLE::high();
	GPS_BACKUP_POWER::high();

	/* Low level init, get SD into MMC mode */
	mmc->Control(CMD_MMC_DETECT_CARD);
	//mmc->Control(CMD_MMC_MOUNT);
	//mmc->Control(CMD_MMC_SET_FILE_BUFFER, &logger_file );

	/* determine the next log file number. */
	for( uint16_t file_number = 0 ; file_number < 999 ; file_number++ )
	{
		nmea_file_name[5] = file_number % 10 + '0';
		nmea_file_name[4] = file_number /10 % 10 + '0';
		nmea_file_name[3] = file_number /100 % 10 + '0';


		// this call will fail if the file does exist.
		//mmc->Control(CMD_MMC_SET_FILENAME, nmea_file_name);
		//mmc->Control(CMD_MMC_SET_ACCESS_RIGHTS, FA_WRITE | FA_CREATE_NEW);

		//f_res = mmc->Open();

		f_res = FR_OK;

		// file opened sucessfully? we are done
		if( f_res == FR_OK )
		{
			break;
		}
		else if( f_res != FR_EXIST )
		{
			LoggerError(f_res);
		}

		if( file_number == 999 )
		{
			LoggerError(f_res);
		}
	}





	while(1)
	{
		/* data aquired stored here */
		char* gps_string = 0;
		uint16_t gps_string_length = 0;

		/* Get more data note this is a blocking call */
		while(gps_string_length == 0)
		{
			gps_string_length = gps->Read(0, gps_string);
			if(gps_string_length != 0)
				break;

			bool _bool;
			m_event.Wait(1,EVENT_FLAG_ANY); /* Blocking call */
			m_event.Clear(1);
		}

		/* Turn LED on to indicate data */
		LED_A_RED::low();

		/* Start timer to turn off LED */
		Timer tmr;
		tmr.Start(false, 100, LoggerLedCallback, 0);

		/* Write data to SD/MMC card */
		//mmc->Write(gps_string_length, gps_string);
		//mmc->Control(CMD_MMC_SYNC);

		/* Turn LED on to indicate data */
		LED_A_BLUE::low();

		/* Start timer to turn off LED */
		//Timer tmr;
		tmr.Start(false, 100, LoggerLedCallback, 0);


		/* Parse data through GPS state machine */
		for( uint16_t index  = 0 ; index < gps_string_length; index++)
		{
		//	flag |= gps_state(gps_string[index]);
		}

		//if(gps_state.changed == true)
		{
			/* Make KML string */
		//	mmc->write();
		}

		/* Somehow handle shutting down thread */
	}


			Thread::Sleep(100);
			/* Power up SD card and access disk */
				MMC_POWER_ENABLE::low();


	Thread::Sleep(1000);

	Scheduler::GetCurrentThread()->Exit();

}


