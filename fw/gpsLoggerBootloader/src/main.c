#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "flash.h"

#include "pFatFs/pff.h"
#include "main.h"

#include "hal.h"

#include <msp430.h>

/* Defines */
#define APP_START_ADDRESS (0x4400)
#define APP_END_ADDRESS (0xE000)
#define PAGE_SIZE (512)
#define APP_PAGES (((APP_END_ADDRESS) - (APP_START_ADDRESS)) / PAGE_SIZE)

#define APP_SEGMENTS (((APP_END_ADDRESS) - (APP_START_ADDRESS)) / 512)
#define SEGMENT_START (APP_START_ADDRESS / 512)

/* Func Prototypes */
FRESULT bootLoadApp(void);
void bootDisplayError(uint8_t errorCount);
void bootDelayMs(uint16_t delayTime);

uint16_t bootValidateApp(void);
void	 bootCleanup();
void	 bootLaunchApp();

/* Local Variables */
FATFS fatFs;

uint16_t crcFile = 0xFFFF;

FRESULT bootLoadApp(void)
{
	FRESULT rc = FR_OK;

	uint16_t memoryAddress = APP_START_ADDRESS;
	uint16_t pages		   = APP_PAGES;
	uint16_t segments = APP_SEGMENTS;
	uint16_t segmentIndex = SEGMENT_START;

	uint8_t page_buffer[PAGE_SIZE];
	uint16_t br;

	/* Erase */
	while(segments--)	{
		if(segments & 1)
		hal_led_b(GREEN);
		else
		hal_led_b(0);

		flashEraseSegment(segmentIndex++);
	}


	hal_led_a(CYAN);
	while (pages--) {
		if(pages & 1)
				hal_led_b(GREEN);
				else
				hal_led_b(0);

		rc = pf_read(page_buffer, sizeof(page_buffer), &br);

		if (rc || br != sizeof(page_buffer)) {
			break;
		}

		flashWriteBlock((uint8_t*)memoryAddress, page_buffer, sizeof(page_buffer));
		memoryAddress += br;
	}

	hal_led_a(GREEN);
	hal_led_b(CYAN);


	return rc;
}

void bootDisplayError(uint8_t errorCount)
{
	uint8_t		   repeatCounter = 3;
	const uint16_t onTime		 = 100;
	const uint16_t offTime		 = 100;
	const uint16_t intervalTime  = 400;
	uint8_t counter;
	hal_led_a(RED);

	for (; repeatCounter > 0; repeatCounter--) {
		for (counter = 0; counter < errorCount; counter++) {
			/* LED ON */
			hal_led_b(RED);
			bootDelayMs(onTime);

			/* LED ON */
			hal_led_b(0);
			bootDelayMs(offTime);
		}
		/* blanking delay */
		bootDelayMs(intervalTime);
	}

	hal_led_a(0);
}

void bootDelayMs(uint16_t delayTime)
{
	while (delayTime--) {
		__delay_cycles(1000 * (MCLK_FREQUENCY / 1000000));
	}
}


#define	POLY	0x1021
unsigned int crc;	// use crc as a global as simple example

void get_crc (unsigned char in) {
unsigned char ctr,temp;

for (ctr=8;ctr>0;--ctr)
{
temp = in ^ (unsigned char)crc; //do next bit
crc >>= 1; //update CRC
if (temp & 0x01) //if LSB XOR == 1
crc ^= POLY; //then XOR polynomial with CRC
in >>= 1; //next bit
}
}




void bootCrcCompute()
{
	crc = 0xFFFF;

	uint8_t* flashPtr = (uint8_t*)APP_START_ADDRESS;
	while (flashPtr < (uint8_t*)APP_END_ADDRESS) {
		get_crc(*flashPtr++);
	}
}




uint16_t bootValidateApp(void)
{
	return 1;

	bootCrcCompute();

	if(crc == 0)
	{
		return 1;
	}
	return 0;
}


FRESULT bootCheckFile(void)
{
	FRESULT rc = FR_OK;

	uint16_t pages		   = APP_PAGES;

	uint8_t page_buffer[PAGE_SIZE];
	uint8_t* page_pointer;
	uint16_t br;

	crc = 0xFFFF;

	hal_led_a(CYAN);
	while (pages--) {
		if(pages & 1)
				hal_led_b(CYAN);
				else
				hal_led_b(0);

		rc = pf_read(page_buffer, sizeof(page_buffer), &br);

		if (rc || br == 0) {
			break;
		}

		page_pointer = page_buffer;

		while(br--)
		{
			get_crc(*page_pointer++);
		}
	}

	crcFile = (uint16_t)page_buffer[PAGE_SIZE-1] << 8 | (uint16_t)page_buffer[PAGE_SIZE-2];

	if(crc != 0)
	{
		rc = FR_NOT_ENABLED;
	}

	return rc;
}


void bootLaunchApp()
{
	/* Take Reset App vector */
	uint16_t resetVector = *(uint16_t*)(APP_END_ADDRESS - 4);


	if (resetVector != 0xFFFF) {

		/* Copy vectors to TOP of RAM */
		uint16_t memoryCounter = 0;
		uint16_t* vectorsRAM = 0x3400 - 0x80;
		uint16_t* vectorsApp = APP_END_ADDRESS - 0x80 - 2;
		for(memoryCounter = 0; memoryCounter < 64; memoryCounter++)
		{
			vectorsRAM[memoryCounter] = vectorsApp[memoryCounter];
		}

		/* Remap Interrupt processor to use vectors at TOP of RAM */
		SYSCTL |= SYSRIVECT;

		/* Assign function pointer to vector value */
		void (*appFn)(void) = (void (*)(void))resetVector;

		appFn(); /* Call app. Note stack is reset as part of low-level init of app. */
	}
}

/*
 * ======== main ========
 */
int main(void)
{
	while (1) {
		
		halInit();

		hal_led_a(GREEN);
		hal_sd_pwr_on();

		hal_led_a(GREEN);
		bootDelayMs(100);

		crcFile = 0;
		uint16_t crcApp = *(uint16_t*)(0xdffe);

		/* Any error breaks out of this block */
		do {
			/* Init SD Media */
			FRESULT rc = pf_mount(&fatFs);
			if (rc != FR_OK) {
				bootDisplayError(rc);
				break;
			}

			SDCard_fastMode();

			rc = pf_open("FIRMWARE.BIN");
			if (rc != FR_OK) {
				bootDisplayError(rc);
				break;
			}

			rc = bootCheckFile(); /* Bad app? Maybe we have an app loaded already */
			if(rc == FR_OK && crcFile != crcApp)
			{
				rc = pf_lseek(0);
				//rc = pf_open("FIRMWARE.BIN"); /* Re-open file */
							if (rc != FR_OK) {
								bootDisplayError(rc);
								break;
							}

					rc = bootLoadApp();
								if (rc != FR_OK) {
									bootDisplayError(rc);
									break;
								}
			}





		} while (0);

		/* Only exit bootloader if there a valid app to run */
		if (bootValidateApp()) {
			bootLaunchApp();
		}

		while(1)
		{
			hal_led_a(RED);
			hal_led_b(0);
			bootDelayMs(200);
			hal_led_a(RED);
			hal_led_b(RED);
			bootDelayMs(500);
		}

	}



}
