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
 * ======== hal.c ========
 *
 */
#include "msp430.h"

#include "driverlib.h"

#include "USB_API/USB_Common/device.h"
#include "USB_config/descriptors.h"

#include "hal.h"

static uint8_t bCompatibiliy = 0;

#define GPIO_ALL	GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3| \
					GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7

/*
 * This function drives all the I/O's as output-low, to avoid floating inputs
 * (which cause extra power to be consumed).  This setting is compatible with  
 * TI FET target boards, the F5529 Launchpad, and F5529 Experimenters Board;  
 * but may not be compatible with custom hardware, which may have components  
 * attached to the I/Os that could be affected by these settings.  So if using
 * other boards, this function may need to be modified.
 */
void USBHAL_initPorts(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN0|GPIO_PIN1); // pullups for Buttons 1 and 2.
    GPIO_selectInterruptEdge(GPIO_PORT_P1,GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION );
    GPIO_clearInterrupt(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN0);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5);

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN4); // GPS RX.
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1|GPIO_PIN2|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1|GPIO_PIN2|GPIO_PIN6|GPIO_PIN7);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1); // disable LEDS
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN4|GPIO_PIN5);

    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1|GPIO_PIN2|GPIO_PIN4);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P6, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

#ifdef __MSP430_HAS_PORT7_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT8_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_ALL);
#endif




    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_ALL);

}

/* Configures the system clocks:
* MCLK = SMCLK = DCO/FLL = mclkFreq (expected to be expressed in Hz)
* ACLK = FLLref = REFO=32kHz
*
* XT2 is not configured here.  Instead, the USB API automatically starts XT2
* when beginning USB communication, and optionally disables it during USB
* suspend.  It's left running after the USB host is disconnected, at which
* point you're free to disable it.  You need to configure the XT2 frequency
* in the Descriptor Tool (currently set to 4MHz in this example).
* See the Programmer's Guide for more information.
*/
void USBHAL_initClocks(uint32_t mclkFreq)
{
	UCS_initClockSignal(
	   UCS_FLLREF,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

	UCS_initClockSignal(
	   UCS_ACLK,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
}



void hal_sd_pwr_on()
{
	 GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
	 GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);

	 /* enable IO pins and spi module */
	 SDCard_init();
	 //disk_initialize(0);     //Attempt to initialize it
}

void hal_sd_pwr_off()
{
	/* assume we've properly shut everything down */
	/* Set all connected IO pins to pull-down (low outputs?) */
	SDCard_deinit();

	 GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
	 GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
}

void hal_gps_pwr_on()
{
	 GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
	 GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);

	 GPS_init();
}

void hal_gps_pwr_off()
{
	/* turn off high side switch */
	GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
	GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);

	GPS_deinit();

	/* disable any high ports to remove leakage. */
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN6 + GPIO_PIN7);
}


void hal_gps_rtc_on()
{
	 GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
	 GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
}

void hal_gps_rtc_off()
{
	 GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
	 GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
}




void hal_compatibility_set()
{
	bCompatibiliy = 1;
}

uint8_t compatibility_switch(uint8_t c)
{
	uint8_t b = c;

	if(bCompatibiliy)
	{
		b &= BLUE;

		if(c & RED)
		{
			b |= GREEN;
		}

		if( c & GREEN)
		{
			b |= RED;
		}
	}

	return b;
}

void hal_led_a( uint8_t c )
{
	c = compatibility_switch(c);


	if(c & 2)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3);
	}

	if(c & 1)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN5);
	}

	if(c & 4)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6);
	}
}


void hal_led_b( uint8_t c )
{


	c = compatibility_switch(c);

	if(c & 2)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7);
	}

	if(c & 1)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
	}

	if(c & 4)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
	}
}




static uint8_t button = 0;

uint8_t hal_button_event()
{
	uint8_t ret_val = button;

	button = 0;
	return ret_val;
}

uint8_t hal_button_status()
{
	if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN0) == GPIO_INPUT_PIN_LOW)
		return 1;
	return 0;
}


uint8_t hal_charge_status()
{
	if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == GPIO_INPUT_PIN_LOW)
		return 1;
	return 0;
}


/*
 * ======== Button_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT0_ISR (void)
#else
#error Compiler not found!
#endif
{
	button |= 1;

	 GPIO_clearInterrupt(GPIO_PORT_P1,GPIO_PIN0);
	 GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN0);

    //Wake from ISR, if sleeping
   __bic_SR_register_on_exit(LPM4_bits);
}
