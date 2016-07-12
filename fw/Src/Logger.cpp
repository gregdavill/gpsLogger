/*
 * Logger.cpp
 *
 *  Created on: 12 Jul 2016
 *      Author: greg
 */

#include "mark3.h"

#include "ff.h"
#include "mmc.h"

#include "IQmathLib.h"


void LoggerError(FRESULT f_res)
{
	while(1)
	{
		/* FLASH LEDS */
		Thread::Sleep(200);
	}
}


void LoggerLedCallback(Thread *_owner, void *_data)
{
	/* Turn off LED */
}

void LoggerMain(void *_fs)
{

	FATFS fatfs;
	FIL logger_file;

	FRESULT f_res;

	unsigned int bytes_written;

	char nmea_file_name[] = "nmea/log000.txt";


	/* Power up SD card and access disk */
	//Hal::SD.PowerOn();
	Thread::Sleep(50);

	/* Low level init, get SD into MMC mode */
	if(detectCard())
	{
		LoggerError(0);
	}

	/* Mount FAT fs */
	f_res = f_mount(&fatfs, "", 0);
	if(f_res)
	{
		LoggerError(f_res);
	}

	/* determine the next log file number. */
	for( uint16_t file_number = 0 ; file_number < 999 ; file_number++ )
	{
		nmea_file_name[10] = file_number % 10 + '0';
		nmea_file_name[9] = file_number /10 % 10 + '0';
		nmea_file_name[8] = file_number /100 % 10 + '0';


		// this call will fail if the file does exist.
		f_res = f_open(&logger_file, nmea_file_name, FA_WRITE | FA_CREATE_NEW);

		// file opened sucessfully? we are done
		if( f_res == FR_OK )
		{
			break;
		}

		if( file_number == 999 )
		{
			LoggerError(f_res);
		}
	}


	f_res = f_write(&logger_file, "test", 4, &bytes_written );
	if( f_res )
	{
		LoggerError(f_res);
	}


    while(1)
    {

    	/* Wait for more data */
    	//m_notify.Wait();

    	/* Turn LED on to indicate data */

    	/* Start timer to turn off LED */
    	Timer tmr;
    	tmr.Start(false, 100, LoggerLedCallback, 0);

    	int gps_day = 1;
    	_IQ2toa(nmea_file_name, "%02.00f", _IQ2(gps_day));
    }
}


