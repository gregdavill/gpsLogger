/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*  
 * ======== main.c ========
 * Mass Storage with SDCARD.
 *
 * This example demonstrates usage of the API with file system software.  
 * It includes an MSP430 port of the open-source "FatFs" software for the FAT 
 * file system.  
 * This example requires hardware with an SD-card interface.  At the time of 
 * writing, the only such hardware TI provdies is the F5529 Experimenter’s 
 * Board, available from TI’s eStore.
 *
 +----------------------------------------------------------------------------+
 * Please refer to the Examples Guide for more details.
 *----------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "flash.h"

//#include "pFatFs/mmc.h"
#include "pFatFs/pff.h"
#include "main.h"


/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"

/* Defines */
#define APP_START_ADDRESS (0xE000)

/* Func Prototypes */


/* Local Variables */
FATFS fatFs;
//FIL firmwareFile;



FRESULT bootLoadApp(void)
{
	FRESULT rc  = FR_OK;

	uint16_t memory_address = APP_START_ADDRESS;

	uint16_t page_buffer[64];
	uint16_t br;

	rc = pf_read(page_buffer, sizeof(page_buffer), br);
	if(rc)
	{
		/* Error condition */
		return rc;
	}
	else if(br != sizeof(page_buffer))
	{
		/* Error condition, possible end of file */
		if(br == 0)
			return FR_OK;

		/* error? */

	}
	else
	{
		/* write data to memory + increment pointer */
		flashWritePage();
		memory_address += br;
	}

	return FR_OK;
}


void bootShortDelay(uint16_t delayTime)
{
	while(delayTime--)
	{
		uint16_t cnt = 4000;
		while(cnt--){
			asm("nop\r\n");
		}
	}
}

/*  
 * ======== main ========
 */
int main (void)
{
	halInit();

	hal_sd_pwr_on();

	hal_led_a(RED);

	bootShortDelay(100);

	hal_led_b(GREEN);

	/* Init SD Media */
	FRESULT rc = pf_mount(&fatFs);

	rc = pf_open("firmware.bin");

	if(rc == FR_OK)
	{
		rc = bootLoadApp();
	}






}
