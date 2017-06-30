/*
 * flash.cpp
 *
 *  Created on: 17 Jun 2017
 *      Author: greg
 */

#include "flash.h"
#include <msp430.h>

#define TOTAL_FLASH_PAGES 32

void flashInit()
{
	/*  */
}

/* Erase all App Flash memory */
void flashErase()
{
	/* Start at segment 8 */
	uint16_t segmentIndexCounter = 8;

	for (; segmentIndexCounter < TOTAL_FLASH_PAGES; segmentIndexCounter++) {
		flashEraseSegment(segmentIndexCounter);
	}
}

// one segment is 512 bytes in size
// Arg: Segment index, 0 = 0x0000, 1 = 0x0200
void flashEraseSegment(uint16_t segmentIndex)
{
	while (FCTL3 & BUSY) {
		// Wait busy
	}

	FCTL3 = FWPW;		  // unlock
	FCTL1 = FWPW | ERASE; // enable erase

	*(uint16_t*)(segmentIndex * 512) = 0; // dummy memory write, start erase

	while (FCTL3 & BUSY) {
		// Wait erase complete, (BUSY)
	}

	FCTL1 = FWPW; // enable erase
	FCTL3 = FWPW + LOCK;
}

void flashWriteBlock(uint8_t* flashMemoryPointer, uint8_t* dataPointer, uint16_t len)
{
	FCTL3 = FWKEY;		 // Clear Lock bit
	FCTL1 = FWKEY + WRT; // Enable byte/word write mode

	while (len--) {
		// test busy
		while (FCTL3 & BUSY) {
		}

		// Write to Flash
		*flashMemoryPointer++ = *dataPointer++;
	}

	FCTL1 = FWKEY;		  // Clear WRT bit
	FCTL3 = FWKEY + LOCK; // Set LOCK bit
}
