#include <fabooh.h>

#include <msp430.h> 
#include <stdint.h>

#include "mark3.h"
//#include "logger.h"
#include "gps.h"

//---------------------------------------------------------------------------
// This block declares the thread data for the main application thread.  It
// defines a thread object, stack (in word-array form), and the entry-point
// function used by the application thread.
#define APP_STACK_SIZE      (320/sizeof(K_WORD))
static Thread  clAppThread;
static K_WORD  awAppStack[APP_STACK_SIZE] __no_init;
static void    AppMain(void *unused_);


#define DYNAMIC_STACK_SIZE     (1300/sizeof(K_WORD))
static Thread  clTask1Thread;
static K_WORD  awTask1Stack[DYNAMIC_STACK_SIZE];
static void    cdcMain(void *unused_);


#define IDLE_STACK_SIZE     (320/sizeof(K_WORD))
static Thread  clIdleThread;
static K_WORD  awIdleStack[IDLE_STACK_SIZE];
static void    IdleMain(void *unused_);


extern void LoggerMain(void *_fs);

EventFlag evtFlg;

msp430GPS gps;

#include <ucs.h>

void SetVcoreUp (unsigned int level)
  {
  	// Open PMM registers for write
  	PMMCTL0_H = PMMPW_H;
  	// Set SVS/SVM high side new level
  	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  	// Set SVM low side to new level
  	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  	// Wait till SVM is settled
  	while ((PMMIFG & SVSMLDLYIFG) == 0);
  	// Clear already set flags
  	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  	// Set VCore to new level
  	PMMCTL0_L = PMMCOREV0 * level;
  	// Wait till new level reached
  	if ((PMMIFG & SVMLIFG))
  		while ((PMMIFG & SVMLVLRIFG) == 0);
  	// Set SVS/SVM low side to new level
  	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  	// Lock PMM registers for write access
  	PMMCTL0_H = 0x00;
  }

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	


    P5SEL |= BIT2+BIT3;                       // Port select XT2
    UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);



    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time..
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);
    //SetVcoreUp (0x03);

   // UCSCTL3 = SELREF_5;                       // Set DCO FLL reference = XT2 (4Mhz)
   // UCSCTL4 |= SELA_2;                        // Set ACLK = REFO






   // PMM_setVCore(PM);


    UCS_initClockSignal(UCS_FLLREF, UCS_XT2CLK_SELECT, UCS_CLOCK_DIVIDER_8);

    UCS_initFLLSettle(4000, 4);

    UCS_initClockSignal(UCS_MCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);
    UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);


    Kernel::Init();

    clAppThread.Init(  awAppStack,  APP_STACK_SIZE,  1, AppMain,  0);
    clIdleThread.Init( awIdleStack, IDLE_STACK_SIZE, 0, IdleMain, 0);

    clAppThread.Start();
    clIdleThread.Start();




    /* Hardware Init */
    LED_A_RED::high();
	LED_A_GREEN::high();
	LED_A_BLUE::high();
	LED_A_RED::setmode_output();
	LED_A_GREEN::setmode_output();
	LED_A_BLUE::setmode_output();


	LED_B_RED::high();
	LED_B_GREEN::high();
	LED_B_BLUE::high();
	LED_B_RED::setmode_output();
	LED_B_GREEN::setmode_output();
	LED_B_BLUE::setmode_output();


	MMC_POWER_ENABLE::setmode_output();
	GPS_POWER_ENABLE::setmode_output();

	MMC_CLK::enable_pupd_resistor();
	MMC_CS::enable_pupd_resistor();
	MMC_MISO::enable_pupd_resistor();
	MMC_MOSI::enable_pupd_resistor();

	gps.Init();

	evtFlg.Init();

    Kernel::Start();


	return 0;
}

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

void rxCallbk(msp430GPS* _gps)
{
	uint8_t d = _gps->Read(0,0);
	evtFlg.Set(d);
}

//---------------------------------------------------------------------------
void AppMain(void *unused_)
{

	/* Implement a state machine to govern the operation of the entire device */
	Thread::Sleep(1000);

	    while(1)
    {
    	if( USB_getConnectionInformation() & USB_VBUS_PRESENT)
    	{
    		LED_A_RED::low();

    		if (clTask1Thread.GetState() == THREAD_STATE_STOP || clTask1Thread.GetState() == THREAD_STATE_EXIT) {
    			clTask1Thread.Init( awTask1Stack, DYNAMIC_STACK_SIZE, 2, cdcMain, 0);
    			clTask1Thread.Start();
    		}

    	}
    	else
    	{
    		if (clTask1Thread.GetState() == THREAD_STATE_STOP || clTask1Thread.GetState() == THREAD_STATE_EXIT) {
				clTask1Thread.Init( awTask1Stack, DYNAMIC_STACK_SIZE, 2, LoggerMain, 0);
				clTask1Thread.Start();
			}

    	}
    }
}

void cdcMain(void *unused_)
{

	GPS_POWER_ENABLE::low();
	Thread::Sleep(100);


		// Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
		   // PMM_setVCore(PMM_CORE_LEVEL_2);
		   // USBHAL_initPorts();           // Config GPIOS for low-power (output low)
		   // USBHAL_initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
		   // initTimer();           // Prepare timer for LED toggling
	USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

	gps.Control(CMD_SET_RX_CALLBACK, rxCallbk, 0,0,0);


	GPS_POWER_ENABLE::high();
	gps.Open();


	    while(1)
	    {
	    	switch(USB_getConnectionState())

	    	{
	    	case ST_ENUM_ACTIVE:
				{
					LED_A_RED::high();


					uint8_t val = evtFlg.Wait(0xFFFF, EVENT_FLAG_ANY);
					evtFlg.Clear(0xFFFF);
					LED_B_GREEN::low();
					if(val != 0)
					{
						USBCDC_sendData(&val, 1, CDC0_INTFNUM);
					}
					LED_B_GREEN::high();
					break;
				}
	    	case ST_PHYS_DISCONNECTED:
			case ST_ENUM_SUSPENDED:
			case ST_PHYS_CONNECTED_NOENUM_SUSP:
				{

					break;
				}

	    	}


	    	if( (USB_getConnectionInformation() & USB_VBUS_PRESENT) == 0 )
			{
				GPS_POWER_ENABLE::low();


				LED_A_RED::high();
				LED_A_GREEN::high();

				Scheduler::GetCurrentThread()->Exit();
			}

	    }
}

//---------------------------------------------------------------------------
void IdleMain(void *unused_)
{
    while(1)
    {
        // Low priority task + power management routines go here.
        // The actions taken in this context must *not* cause the thread
        // to block, as the kernel requires that at least one thread is
        // schedulable at all times when not using an idle thread.

        // Note that if you have no special power-management code or idle
        // tasks, an empty while(1){} loop is sufficient to guarantee that
        // condition.
    	LPM0;
    }
}
