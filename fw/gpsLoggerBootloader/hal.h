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
 * ======== hal.h ========
 *
 * Device and board specific pins need to be configured here
 *
 */

/*----------------------------------------------------------------------------
 * The following function names are deprecated.  These were updated to new 
 * names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/

#ifndef DEPRECATED
#define   initPorts       USBHAL_initPorts
#define   initClocks      USBHAL_initClocks
#endif


enum colours{
	RED 	= 2,
	GREEN 	= 1,
	BLUE 	= 4,

	YELLOW = 	RED |	GREEN ,
	CYAN = 				GREEN |	BLUE,
	MAGENTA = 	RED |			BLUE,
	WHITE = 	RED |	GREEN |	BLUE
};

#define MCLK_FREQUENCY  8000000


void halInit(void);

void USBHAL_initPorts(void);
void USBHAL_initClocks(uint32_t mclkFreq);

void hal_led_a( uint8_t c );
void hal_led_b( uint8_t c );


void hal_gps_pwr_on(void);
void hal_gps_pwr_off(void);

void hal_sd_pwr_on(void);
void hal_sd_pwr_off(void);

uint8_t hal_button_event(void);


//Released_Version_5_00_01
