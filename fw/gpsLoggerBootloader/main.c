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


#include "driverlib.h"


#include "FatFs/mmc.h"
#include "FatFs/ff.h"
#include "main.h"


/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"


/* Func Prototypes */
void initTimer(void);
void setTimer_A_Parameters(void);

volatile uint8_t bDelayDone;
Timer_A_initUpModeParam Timer_A_params = {0};

FATFS fatFs;
FIL firmwareFile;


/*  
 * ======== main ========
 */
int main (void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    //PMM_setVCore(PMM_CORE_LEVEL_2);
	USBHAL_initPorts();                // Config GPIOS for low-power (output low)
	//USBHAL_initClocks(MCLK_FREQUENCY); // Config clocks. MCLK=SMCLK=FLL=MCLK_FREQUENCY; ACLK=REFO=32kHz


    initTimer();

	hal_sd_pwr_on();

	hal_led_a(0);
	hal_led_b(0);

	Timer_A_stop(TIMER_A0_BASE);

	__enable_interrupt();       // Enable interrupts globally


	/* Init SD Media */
	FRESULT rc = f_mount(&fatFs,"", 0);

	rc = f_open(&firmwareFile, "firmware.bin", FA_READ);

	while(1)
	{
		uint16_t memory_address = 0x4400;

		uint16_t page_buffer[64];
		uint16_t br;

		rc = f_read(&firmwareFile, page_buffer, sizeof(page_buffer), br);
		if(rc)
		{
			/* Error condition */
			break;
		}
		else if(br != sizeof(page_buffer))
		{
			/* Error condition, possible end of file */
			if(br == 0)
				break;
		}
		else
		{
			/* write data to memory + increment pointer */
			FlashCtl_write16(page_buffer, (uint16_t*)memory_address, br);
			memory_address += br;
		}



	}






}

/*  
 * ======== TIMER0_A0_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
    //Set the flag that will trigger main() to detect the card
   // bDetectCard = 0x01;

    //Wake from ISR, if sleeping
    __bic_SR_register_on_exit(LPM0_bits);
}


/*
 * ======== TIMER0_A0_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
    //Set the flag that will trigger main() to detect the card
    bDelayDone = 0x01;

    //Wake from ISR, if sleeping
    __bic_SR_register_on_exit(LPM3_bits);
}

/*  
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
    }
}


/*
 * ======== setTimer_A_Parameters ========
 */
// This function sets the timer A parameters
void setTimer_A_Parameters()
{
    Timer_A_params.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	Timer_A_params.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    Timer_A_params.timerPeriod = 32768;
	Timer_A_params.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	Timer_A_params.captureCompareInterruptEnable_CCR0_CCIE =
		       TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
	Timer_A_params.timerClear = TIMER_A_DO_CLEAR;
	Timer_A_params.startTimer = 0;
}


void initTimer(void)
{

    setTimer_A_Parameters();   

    //start timer
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);

    Timer_A_initUpMode(TIMER_A0_BASE, &Timer_A_params);

    Timer_A_startCounter(TIMER_A0_BASE,
        TIMER_A_UP_MODE);

}


void delay_ms(uint16_t time)
{

	/* Setup timer */
	Timer_A_initUpModeParam tParams;
	tParams.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	tParams.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	tParams.timerPeriod = time * 32;
	tParams.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	tParams.captureCompareInterruptEnable_CCR0_CCIE =
		       TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
	tParams.timerClear = TIMER_A_DO_CLEAR;
	tParams.startTimer = 0;


    /* Start timer */
    Timer_A_clearTimerInterrupt(TIMER_A1_BASE);

    Timer_A_initUpMode(TIMER_A1_BASE, &tParams);

    Timer_A_startCounter(TIMER_A1_BASE,
        TIMER_A_UP_MODE);


    /* Interupt will wake us */
    bDelayDone = 0;
    while( bDelayDone == 0)
    	LPM3;

    Timer_A_stop(TIMER_A1_BASE);

}


