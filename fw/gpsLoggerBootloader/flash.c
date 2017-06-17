/*
 * flash.cpp
 *
 *  Created on: 17 Jun 2017
 *      Author: greg
 */



#include "flash.h"
#include <msp430.h>


void flashInit()
{
	/*  */
}

void flashEraseSegment(uint16_t segmentAddress)
{
	while(FCTL3 & BUSY);  // Wait busy

	FCTL3 = FWPW; // unlock
	FCTL1 = FWPW|ERASE; // enable erase

	*(uint16_t*)segmentAddress = 0; // dummy memory write, start erase

	while(FCTL3 & BUSY);  // Wait erase complete, (BUSY)

	FCTL3 = FWPW+LOCK;


}

void flashWritePage(uint16_t* dataPointer, uint16_t* memoryPointer, uint16_t len)
{
	FlashCtl_write16(dataPointer, memoryPointer, len);
}
