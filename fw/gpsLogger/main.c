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

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_MSC_API/UsbMscScsi.h"
#include "USB_API/USB_MSC_API/UsbMsc.h"
#include "USB_API/USB_MSC_API/UsbMscStateMachine.h"

#include "USB_app/msc.h"  // FatFs #includes
#include "USB_app/FatFs/mmc.h"
#include "main.h"

#include "gps.h"

/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"


/* Func Prototypes */
void initTimer(void);
void setTimer_A_Parameters(void);




void func_StateIdle(void);
void entry_StateIdle(void);
void func_StateLogging(void);
void entry_StateLogging(void);
void func_StateUsbConnected(void);
void entry_StateUsbConnected(void);



// Global flag by which the timer ISR will trigger main() to check the
// media status
volatile uint8_t bDelayDone;
Timer_A_initUpModeParam Timer_A_params = {0};


#define GPS_MODE 1

enum eOperatingStates {e_StateIdle = 0, e_StateLogging = 1, e_StateUsbConnected = 2, e_StateLast};

enum eOperatingStates OperatingState = 0, NewState = 0;


typedef ((*funcPtr_t)(void));


funcPtr_t StateFuctions[e_StateLast] = {
		&func_StateIdle, &func_StateLogging, &func_StateUsbConnected
};

funcPtr_t StateEntryFuctions[e_StateLast] = {
		&entry_StateIdle, &entry_StateLogging, &entry_StateUsbConnected
};


/*  
 * ======== main ========
 */
int main (void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    PMM_setVCore(PMM_CORE_LEVEL_2);
	USBHAL_initPorts();                // Config GPIOS for low-power (output low)
	USBHAL_initClocks(MCLK_FREQUENCY); // Config clocks. MCLK=SMCLK=FLL=MCLK_FREQUENCY; ACLK=REFO=32kHz


    initTimer();
    entry_StateIdle();

    // state machine
    while (1)
    {
    	StateFuctions[(uint16_t)OperatingState]();

    	/* Call entry fuction if required */
    	if(NewState != OperatingState)
    	{
    		StateEntryFuctions[(uint16_t)NewState]();

    		/* Update state */
    		OperatingState = NewState;
    	}
    }

}

/* IDLE state */

void func_StateIdle(void)
{

	/* If we are running then the following cases apply:
	 *  - Battery has been plugged in
	 *  - State has been set to IDLE (turn OFF)
	 *  - We have been plogged into USB (turn ON USB MSC)
	 *  - Our button has been pressed (Turn ON Logging)
	 *
	 */

	/* Check and Handle the events */

	/* USB connected */
	if(USB_getConnectionInformation() & USB_VBUS_PRESENT)
	{
		/* Visual Debug of USB connection */
		hal_led_a(CYAN);

		NewState = e_StateUsbConnected;
		return;

//PMM_setVCore(PMM_CORE_LEVEL_2);
		/* Power up SD card*/
		//hal_sd_pwr_on();
		//delay_ms(100);


		//USBMSC_initMSC();                  // Initialize MSC API, and report media to the host
		if (USB_enable() == USB_SUCCEED){

			//hal_led_a(GREEN);
			//hal_sd_pwr_on();
			//detectCard();

			USB_reset();
			USB_connect();  //generate rising edge on DP -> the host enumerates our device as full speed device
		}
		return; // Exit, next loop around will be different state
	}

	/* start GPS */
	if(hal_button_event())
	{

		/* delay for starting */
		hal_led_a(RED);
		uint8_t timeout = 16;
		while( hal_button_status() == 1 && --timeout )
		{
			delay_ms(100);
		}
		hal_led_a(0);
		if( hal_button_status() == 0 )
			return;

		NewState = e_StateLogging;
		hal_led_a(CYAN);

		return; // don't enter sleep
	}


	USB_disable(); //Disable
	hal_gps_rtc_on(); // saves around 7uA
	Timer_A_stop(TIMER_A0_BASE);
	//UCS_turnOffSMCLK();
	//PMM_setVCore(PMM_CORE_LEVEL_0);
	__bis_SR_register(LPM4_bits + GIE);
	_NOP();
	//UCS_turnOnSMCLK();
	//PMM_setVCore(PMM_CORE_LEVEL_2);
}

/* This function is called during the transition to StateIdle */
void entry_StateIdle(void)
{
	hal_led_a(0);
	hal_led_b(0);

	USB_setup(FALSE, TRUE);      // Init USB & events; if a host is present, connect


	gps_stop();

	hal_gps_pwr_off();
	hal_sd_pwr_off();
	UCS_turnOffXT2();

	USB_disable(); //Disable
	hal_gps_rtc_on(); // saves around 7uA
	Timer_A_stop(TIMER_A0_BASE);

	__enable_interrupt();       // Enable interrupts globally
}

void func_StateLogging(void)
{
	/* Do we need to stop logging? */

	/* Have we connected to USB, Go via IDLE state to get system setup. */
	if((USB_getConnectionInformation() & USB_VBUS_PRESENT))
	{
		NewState = e_StateIdle;
		return;
	}

	if(hal_button_event())
	{
		/* delay for stopping */
		uint8_t timeout = 16;

		while( hal_button_status() == 1 && --timeout )
		{
			hal_led_a(RED);
			delay_ms(100);
			hal_led_a(0);
		}
		hal_led_a(0);

		if( hal_button_status() == 0 )
			return;

		NewState = e_StateIdle;
		return;
	}


	while( gps_check() )
	{
		gps_do();
	}

	/* Wait til new data */
	__bis_SR_register(LPM0_bits + GIE);
	_NOP();

}

/* This function is called during the transition to StateIdle */
void entry_StateLogging(void)
{
	hal_led_a(0);
	hal_led_b(0);
	hal_sd_pwr_on();
	delay_ms(100);

	detectCard();


	hal_gps_pwr_on();
	hal_gps_rtc_on(); // saves around 7uA

	gps_start();
}


void func_StateUsbConnected(void)
{
	/* No USB present? Why are we here, What does it all mean? */
	if(!(USB_getConnectionInformation() & USB_VBUS_PRESENT))
	{
		NewState = e_StateIdle;
		return;
	}

	/* check state of chareger? */
	if( hal_charge_status())
	hal_led_b(RED);
	else
	hal_led_b(GREEN);

	hal_button_event(); // clear button event
	switch (USB_getConnectionState())
	{
		case ST_ENUM_ACTIVE:



			USBMSC_processMSCBuffer(); // Handle READ/WRITE cmds from the host

			// Every second, the Timer_A ISR sets this flag.  The
			// checking can't be done from within the timer ISR, because it
			// enables interrupts, and this is not a recommended
			// practice due to the risk of nested interrupts.
			//if (bDetectCard){
			//USBMSC_checkMSCInsertionRemoval();

			// Clear the flag, until the next timer ISR
			//bDetectCard = 0x00;
			//}

			break;

		// These cases are executed while your device is disconnected from
		// the host (meaning, not enumerated); enumerated but suspended
		// by the host, or connected to a powered hub without a USB host
		// present.
		case ST_PHYS_DISCONNECTED:
		case ST_ENUM_SUSPENDED:
		case ST_PHYS_CONNECTED_NOENUM_SUSP:
			hal_led_a(BLUE);
			//state = sIDLE;
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


void entry_StateUsbConnected(void)
{

	hal_sd_pwr_on();
	delay_ms(100);

	 USB_setup(TRUE, TRUE);      // Init USB & events; if a host is present, connect

	 USBMSC_initMSC();                  // Initialize MSC API, and report media to the host

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


