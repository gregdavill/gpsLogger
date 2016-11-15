#include <fabooh.h>

#include <msp430.h> 
#include <stdint.h>

#include "mark3.h"
#include "Message.h"
#include "dvrPIN.h"
//#include "logger.h"

#include "mainEvent.h"

#include "gps.h"


#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"



//---------------------------------------------------------------------------
// This block declares the thread data for the main application thread.  It
// defines a thread object, stack (in word-array form), and the entry-point
// function used by the application thread.
#define APP_STACK_SIZE      (512/sizeof(K_WORD))
static Thread  clAppThread;
static K_WORD  awAppStack[APP_STACK_SIZE];
static void    AppMain(void *unused_);

#define DYNAMIC_STACK_SIZE     (512/sizeof(K_WORD))
static Thread  clTask1Thread;
static K_WORD  awTask1Stack[DYNAMIC_STACK_SIZE];
static void    cdcMain(void *unused_);

#define IDLE_STACK_SIZE     (120/sizeof(K_WORD))
static Thread  clIdleThread;
static K_WORD  awIdleStack[IDLE_STACK_SIZE];
static void    IdleMain(void *unused_);


extern void LoggerMain(void *_fs);

EventFlag evtFlg;
EventFlag MainEvent;

MessageQueue eventQ;

msp430GPS gps;

MSP430pin pin;

#include <ucs.h>

void start_usb_boot_loader(void)
{
    typedef void (*functionpointer)(void);
    const functionpointer bsloader = (functionpointer)(0x1000);
    USB_disconnect();
    USB_disable();
    SYSBSLC &= ~(SYSBSLPE);
    __disable_interrupt();
    bsloader();          // won't return
}

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

	V_SENSE_ENABLE::low();
	V_SENSE_ENABLE::setmode_output();

	/* Inputs */
	POWER_BUTTON::enable_pupd_resistor();
	POWER_BUTTON::high();
	BATTERY_CHARGE_STATUS::enable_pupd_resistor();
	BATTERY_CHARGE_STATUS::high();

	gps.SetName("/dev/gps");			//!< Add the serial driver
	gps.Init();

	DriverList::Add( &gps );

	evtFlg.Init();
	MainEvent.Init();

	eventQ.Init();

	pin.Init();
	pin.Control(CMD_SET_QUEUE, &eventQ, 0,0,0);

    Kernel::Start();


	return 0;
}


void rxCallbk(msp430GPS* _gps)
{
	uint8_t d = _gps->Read(0,0);
	evtFlg.Set(d);
}


void led_off(Thread *owner, void *pvData_)
{

	LED_A_RED::high();
	LED_A_GREEN::high();
	LED_A_BLUE::high();
	LED_B_RED::high();
	LED_B_GREEN::high();
	LED_B_BLUE::high();
}

uint16_t calcBattery();

//---------------------------------------------------------------------------
void AppMain(void *unused_)
{

	/* Implement a state machine to govern the operation of the entire device */

	Timer a;
	a.Init();
	//a.Start(0, 150, disp_battery, &a);

	static uint8_t value = 0;

			value = calcBattery();

		while(value--)
		{
			LED_A_GREEN::low();
			Thread::Sleep(100);
			LED_A_GREEN::high();
			Thread::Sleep(200);
		}


	static bool bLogger = false;

	while(1)
	{
		/* Get message */
		Message* msg = eventQ.Receive();

		switch(msg->GetCode())
		{
		case 0x8101:
			{
				// power button	DOWN
				LED_A_GREEN::low();
			}
		break;

		case 0x8102:
			{
				// power button	UP
				LED_A_GREEN::high();
				LED_B_BLUE::high();
				LED_B_GREEN::high();

				if(bLogger == true)
				{
					clTask1Thread.Init( awTask1Stack, DYNAMIC_STACK_SIZE, 2, cdcMain, &eventQ);
					clTask1Thread.Start();
				}
				else
				{
					USB_disconnect();
					    USB_disable();
					MMC_POWER_ENABLE::low();
					GPS_POWER_ENABLE::low();
					clTask1Thread.Stop();
				}


			}
		break;

		case 0x8103:
		{

			// power button	HOLD
			if((uint16_t)msg->GetData() == 200)
			{

				{
					LED_B_GREEN::low();
					bLogger ^= true;
				}
			//a.Start(0, 150, led_off, 0);

			}
			else if( (uint16_t)msg->GetData() < 200)
			{
				LED_B_GREEN::toggle();
			}
		}
			break;
		default:
			break;
		}

		/* return message */
		GlobalMessagePool::Push(msg);
	}



	if( USB_getConnectionInformation() & USB_VBUS_PRESENT )
		MainEvent.Set(USB_EVENT_VBUS_ON);
	else
		MainEvent.Set(USB_EVENT_VBUS_OFF);

	    while(1)
    {
	    MainEvent_t event = (MainEvent_t)MainEvent.Wait(0xFFFF, EVENT_FLAG_ANY);

	    switch(event)
	    {
			case USB_EVENT_VBUS_ON: {

				LED_A_RED::low();

				if (clTask1Thread.GetState() == THREAD_STATE_STOP || clTask1Thread.GetState() == THREAD_STATE_EXIT) {
					MainEvent.Clear(USB_EVENT_VBUS_ON);
					clTask1Thread.Init( awTask1Stack, DYNAMIC_STACK_SIZE, 2, cdcMain, 0);
					clTask1Thread.Start();
				}

				break;
			}

			case USB_EVENT_VBUS_OFF: {


				if (clTask1Thread.GetState() == THREAD_STATE_STOP || clTask1Thread.GetState() == THREAD_STATE_EXIT) {
					MainEvent.Clear(USB_EVENT_VBUS_OFF);
					clTask1Thread.Init( awTask1Stack, DYNAMIC_STACK_SIZE, 2, LoggerMain, 0);
					clTask1Thread.Start();
				}
				break;
			}

			default: {
				MainEvent.Clear(0xFFFF);
				break;
			}
	    }

    }
}


uint16_t calcBattery()
{


	// Setup ComparatorB

	  CBCTL0 |= CBIPEN + CBIPSEL_0; // Enable V+, input channel CB0
	  CBCTL1 |= CBPWRMD_0 | CBMRVS;          // CBMRVS=0 => select VREF1 as ref when CBOUT
	                                // is high and VREF0 when CBOUT is low
	                                // High-Speed Power mode
	  CBCTL2 = CBRSEL;             // VRef is applied to -terminal
	  CBCTL2 |= CBRS_1;
	  CBCTL3 |= BIT0;               // Input Buffer Disable @P6.0/CB0
	  CBCTL1 |= CBON;               // Turn On ComparatorB

	  V_SENSE_ENABLE::high();


	  uint8_t ref_voltage = 0;

	  CBINT = 0;

	  for( ref_voltage = 0 ; ref_voltage < 32; ref_voltage++ )
	  {
		  uint16_t old_cbctl2 = CBCTL2;
		  old_cbctl2 &= ~(CBREF0_31 | CBREF1_31);
		  old_cbctl2 |= (ref_voltage | ref_voltage << 8);

		  CBCTL2 = old_cbctl2;

		  __delay_cycles(75);           // delay for the reference to settle


		  if(CBINT & CBIIFG)
			  break;

	  }

	  CBCTL3 &= ~BIT0;               // Input Buffer Disable @P6.0/CB0
	  CBCTL1 &= ~CBON;

	  V_SENSE_ENABLE::low();


	 if( ref_voltage <= 16 )
	 {
		 ref_voltage = 0;
	 }
	 else
	 {
		 ref_voltage -= 16;
	 }

	 return ref_voltage;

}



void cdcMain(void *unused_)
{

		// Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
		   // PMM_setVCore(PMM_CORE_LEVEL_2);
		   // USBHAL_initPorts();           // Config GPIOS for low-power (output low)
		   // USBHAL_initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
		   // initTimer();           // Prepare timer for LED toggling

	gps.Control(CMD_SET_RX_CALLBACK, rxCallbk, 0,0,0);



	GPS_POWER_ENABLE::high();
	gps.Open();

	USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

	gps.Control(CMD_SET_RX_ENABLE, 0, 0,0,0);

	uint8_t buff[32];

	uint8_t counter = 0;

	    while(1)
	    {
	    	switch(USB_getConnectionState())

	    	{
	    	case ST_ENUM_ACTIVE:
				{
					uint8_t c = gps.Read(32, buff);

					if( c != 0)
						USBCDC_sendData(buff, c ,CDC0_INTFNUM);
					else
						Thread::Sleep(100);
					//USBCDC_sendDataAndWaitTillDone (&c, 1, CDC0_INTFNUM, 0);

					break;
				}
	    	case ST_PHYS_DISCONNECTED:
			case ST_ENUM_SUSPENDED:
			case ST_PHYS_CONNECTED_NOENUM_SUSP:
				{
					Thread::Sleep(100);
					break;
				}

	    	}


	    	if( (USB_getConnectionInformation() & USB_VBUS_PRESENT) == 0 )
			{
				GPS_POWER_ENABLE::low();


				LED_A_RED::high();
				LED_A_GREEN::high();

				//Scheduler::GetCurrentThread()->Exit();
			}
	    	else
	    	{
	    		GPS_POWER_ENABLE::high();
	    		GPS_BACKUP_POWER::high();

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
