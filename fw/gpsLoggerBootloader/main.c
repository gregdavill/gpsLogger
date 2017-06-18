#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "flash.h"

#include "pFatFs/pff.h"
#include "main.h"

#include "hal.h"

/* Defines */
#define APP_START_ADDRESS (0xE000)

/* Func Prototypes */
FRESULT bootLoadApp(void);
void bootDisplayError(uint8_t errorCount);
void bootDelayMs(uint16_t delayTime);
uint16_t bootValidateApp(void);
void	 bootCleanup();
void	 bootLaunchApp();

/* Local Variables */
FATFS fatFs;


FRESULT bootLoadApp(void)
{
	FRESULT rc = FR_OK;

	uint16_t memory_address = APP_START_ADDRESS;

	uint16_t page_buffer[64];
	uint16_t br;

	rc = pf_read(page_buffer, sizeof(page_buffer), br);
	if (rc) {
		/* Error condition */
		return rc;
	} else if (br != sizeof(page_buffer)) {
		/* Error condition, possible end of file */
		if (br == 0)
			return FR_OK;

		/* error? */

	} else {
		/* write data to memory + increment pointer */
		flashWritePage();
		memory_address += br;
	}

	return FR_OK;
}

void bootDisplayError(uint8_t errorCount)
{
	uint8_t		   repeatCounter = 3;
	const uint16_t onTime		 = 100;
	const uint16_t offTime		 = 100;
	const uint16_t intervalTime  = 400;

	hal_led_a(RED);

	for (; repeatCounter > 0; repeatCounter--) {
		for (uint8_t counter = 0; counter < errorCount; counter++) {
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

uint16_t bootValidateApp(void)
{
	void(*appFn)(void) = (void (*)(void))APP_START_ADDRESS;

	if (appFn != 0) {
		return 1;
	}
	return 0;
}

void bootCleanup()
{
}

void bootLaunchApp()
{
	bootCleanup();

	void(*appFn)(void) = (void (*)(void))APP_START_ADDRESS;

	if (appFn != 0) {
		appFn();
	}
}

/*
 * ======== main ========
 */
int main(void)
{
	while (1) {
		halInit();

		hal_sd_pwr_on();

		hal_led_a(GREEN);

		bootDelayMs(100);

		hal_led_b(GREEN);

		/* Any error breaks out of this block */
		do {
			/* Init SD Media */
			FRESULT rc = pf_mount(&fatFs);
			if (rc != FR_OK) {
				bootDisplayError(rc);
				break;
			}

			rc = pf_open("FIRMWARE.BIN");
			if (rc != FR_OK) {
				bootDisplayError(rc);
				break;
			}

			rc = bootLoadApp();
			if (rc != FR_OK) {
				bootDisplayError(rc);
				break;
			}

		} while (0);

		/* Only exit bootloader if there a valid app to run */
		if (bootValidateApp()) {
			break;
		}
	}

	bootCleanup();

	bootLaunchApp();
}
