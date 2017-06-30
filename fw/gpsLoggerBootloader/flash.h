/*
 * flash.h
 *
 *  Created on: 17 Jun 2017
 *      Author: greg
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>

void flashInit(void);
void flashEraseSegment(uint16_t segmentAddress);
void flashWriteBlock(uint8_t* dataPointer, uint8_t* flashMemoryPointer, uint16_t len);

#endif /* FLASH_H_ */
