#include <msp430.h> 
#include <stdint.h>


#include "mark3.h"

//---------------------------------------------------------------------------
// This block declares the thread data for the main application thread.  It
// defines a thread object, stack (in word-array form), and the entry-point
// function used by the application thread.
#define APP_STACK_SIZE      (320/sizeof(K_WORD))
static Thread  clAppThread;
static K_WORD  awAppStack[APP_STACK_SIZE];
static void    AppMain(void *unused_);

//---------------------------------------------------------------------------
// This block declares the thread data for the idle thread.  It defines a
// thread object, stack (in word-array form), and the entry-point function
// used by the idle thread.
#define IDLE_STACK_SIZE     (320/sizeof(K_WORD))
static Thread  clIdleThread;
static K_WORD  awIdleStack[IDLE_STACK_SIZE];
static void    IdleMain(void *unused_);


/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    Kernel::Init();

    clAppThread.Init(  awAppStack,  APP_STACK_SIZE,  1, AppMain,  0);
    clIdleThread.Init( awIdleStack, IDLE_STACK_SIZE, 0, IdleMain, 0);

    clAppThread.Start();
    clIdleThread.Start();

    Kernel::Start();


	return 0;
}

//---------------------------------------------------------------------------
void AppMain(void *unused_)
{

	/* Implement a state machine to govern the operation of the entire device */
	Thread::Sleep(100);
	Hal::LedA::Toggle();

    while(1)
    {
    	Thread::Sleep(100);
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
    }
}
