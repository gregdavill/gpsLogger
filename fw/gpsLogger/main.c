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

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_MSC_API/UsbMscScsi.h"
#include "USB_API/USB_MSC_API/UsbMsc.h"
#include "USB_API/USB_MSC_API/UsbMscStateMachine.h"

#include "USB_app/msc.h"  // FatFs #includes
#include "main.h"

/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"



void initTimer(void);
void setTimer_A_Parameters(void);

// Global flag by which the timer ISR will trigger main() to check the
// media status
extern uint8_t bDetectCard;
Timer_A_initUpModeParam Timer_A_params = {0};


#define GPS_MODE 1

uint8_t mode = GPS_MODE;

/*  
 * ======== main ========
 */
void main (void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
    PMM_setVCore(PMM_CORE_LEVEL_2);
    USBHAL_initPorts();                // Config GPIOS for low-power (output low)
    USBHAL_initClocks(MCLK_FREQUENCY); // Config clocks. MCLK=SMCLK=FLL=MCLK_FREQUENCY; ACLK=REFO=32kHz
    USBMSC_initMSC();                  // Initialize MSC API, and report media to the host
    initTimer();
    USB_setup(TRUE, TRUE);      // Init USB & events; if a host is present, connect

    __enable_interrupt();       // Enable interrupts globally

    GPS_init();

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN7);



    while (1)
    {
    	if(gps_check())
    	{
    		gps_do();
    	}
    	else
    	{
			switch (USB_getConnectionState())
			{
				case ST_ENUM_ACTIVE:

					GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);

					USBMSC_processMSCBuffer(); // Handle READ/WRITE cmds from the host

					// Every second, the Timer_A ISR sets this flag.  The
					// checking can't be done from within the timer ISR, because it
					// enables interrupts, and this is not a recommended
					// practice due to the risk of nested interrupts.
					if (bDetectCard){
						USBMSC_checkMSCInsertionRemoval();

						// Clear the flag, until the next timer ISR
						bDetectCard = 0x00;
					}

					break;

				// These cases are executed while your device is disconnected from
				// the host (meaning, not enumerated); enumerated but suspended
				// by the host, or connected to a powered hub without a USB host
				// present.
				case ST_PHYS_DISCONNECTED:
				case ST_ENUM_SUSPENDED:
				case ST_PHYS_CONNECTED_NOENUM_SUSP:
					GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // disable status LED
					__bis_SR_register(LPM3_bits + GIE);
					_NOP();
					break;

				// The default is executed for the momentary state
				// ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
				// seconds.  Be sure not to enter LPM3 in this state; USB
				// communication is taking place here, and therefore the mode must
				// be LPM0 or active-CPU.
				case ST_ENUM_IN_PROGRESS:
				default:;
			}
    	}
    }  //while(1)

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
    bDetectCard = 0x01;

    //Wake from ISR, if sleeping
    __bic_SR_register_on_exit(LPM0_bits);
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
            USB_disable(); //Disable
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
	Timer_A_params.startTimer = false;
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
//Released_Version_5_00_01
