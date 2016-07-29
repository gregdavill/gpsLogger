/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2012-2015 Funkenstein Software Consulting, all rights reserved.
See license.txt for more information
===========================================================================*/
/*!

    \file   kerneltimer.cpp

    \brief  Kernel Timer Implementation for ATMega328p
*/

#include "kerneltypes.h"
#include "kerneltimer.h"
#include "mark3cfg.h"

#include <msp430.h>


//---------------------------------------------------------------------------
void KernelTimer::Config(void)
{
    TA0CTL = 0;                        // Reset the register
    TA0R = 0;                          // Clear Timer A
    TA0CTL |= TACLR;                   // Reset the clock divider, etc.
    TA0CTL |= TASSEL_1;                // Set the timer to use SMCLK
    TA0CTL &= ~TAIFG;                  // Clear any pending interrupts
    TA0CCTL0 &= ~CCIFG;                // Clear pending interrupts
#if KERNEL_TIMERS_TICKLESS
    TA0CTL |= ID_0; // Divide-by-8
#else
    TA0CCR0 = (K_USHORT)TIMER_FREQ; // Set interrupts to occur at tick freq.
#endif
}

//---------------------------------------------------------------------------
void KernelTimer::Start(void)
{
    TA0R = 0;

    TA0CTL &= ~TAIFG;                // Clear any pending interrupt on timer A
    TA0CCTL0 &= ~CCIFG;              // Clear pending interrupts on the

    TA0CCTL0 |= CCIE;                // Enable interrupts on TimerA0 CCR

    TA0CTL |= MC_1;                  // Set timer mode to count to TACCR0

}

//---------------------------------------------------------------------------
void KernelTimer::Stop(void)
{
	TA0CTL &= ~MC_1;

    TA0CCTL0 &= ~CCIE;               // Disable TA0 CCR

    TA0R=0;
    TA0CCR0=0;
}

//---------------------------------------------------------------------------
K_USHORT KernelTimer::Read(void)
{
#if KERNEL_TIMERS_TICKLESS
	K_USHORT usVal;
	TA0CCTL0 &= ~MC_1;
	usVal = TA0R;
	TA0CCTL0 |= MC_1;
	return usVal;
#else
	return 0;
#endif
}

//---------------------------------------------------------------------------
K_ULONG KernelTimer::SubtractExpiry(K_ULONG ulInterval_)
{
#if KERNEL_TIMERS_TICKLESS
	TA0CCR0 -= ulInterval_;
	return (K_ULONG)TA0CCR0;
#else
    return 0;
#endif
}

//---------------------------------------------------------------------------
K_ULONG KernelTimer::TimeToExpiry(void)
{
#if KERNEL_TIMERS_TICKLESS
	K_USHORT usCurrent = KernelTimer::Read();
	K_USHORT usMax = TA0CCR0;
	if (usMax >= usCurrent)
	{
		return 0;
	}
	return (usMax - usCurrent);
#else
    return 0;
#endif
}

//---------------------------------------------------------------------------
K_ULONG KernelTimer::GetOvertime(void)
{
#if KERNEL_TIMERS_TICKLESS
    (K_ULONG)KernelTimer::Read();
#else
    return 0;
#endif
}

//---------------------------------------------------------------------------
K_ULONG KernelTimer::SetExpiry(K_ULONG ulInterval_)
{
#if KERNEL_TIMERS_TICKLESS
	K_ULONG ulRet;
	if (ulInterval_ >= 65535)
	{
		ulRet = 65535;
	}
	else
	{
		ulRet = ulInterval_;
	}

	TA0CCR0 = (K_USHORT)ulRet;
	TA0CCTL0 |= CCIE;                // Enable interrupts on TimerA0 CCR
	TA0CTL |= MC_1;

	return ulRet;
#else
	return 0;
#endif
}

//---------------------------------------------------------------------------
void KernelTimer::ClearExpiry(void)
{
#if KERNEL_TIMERS_TICKLESS
    TA0CCR0 = 65535;
#endif
}

//---------------------------------------------------------------------------
K_UCHAR KernelTimer::DI(void)
{
#if KERNEL_TIMERS_TICKLESS
	K_UCHAR ucRet = ((TA0CCTL0 & CCIE) != 0);
	TA0CCTL0 &= ~CCIE;
	TA0CCTL0 &= ~CCIFG;
	return ucRet;
#else
    return 0;
#endif
}

//---------------------------------------------------------------------------
void KernelTimer::EI(void)
{
    KernelTimer::RI(1);
}

//---------------------------------------------------------------------------
void KernelTimer::RI(K_BOOL bEnable_)
{
#if KERNEL_TIMERS_TICKLESS
	if (bEnable_)
	{
		TA0CCTL0 |= CCIE;
	}
	else
	{
		TA0CCTL0 &= ~CCIE;
	}
#endif
}
