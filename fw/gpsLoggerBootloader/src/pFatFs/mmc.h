#ifndef MMC_H__
#define MMC_H__

#include <stdint.h>

void init_spi (void);		/* Initialize SPI port (asmfunc.S) */
void deselect (void);		/* Select MMC (asmfunc.S) */
void select (void);			/* Deselect MMC (asmfunc.S) */
void xmit_spi (uint8_t d);		/* Send a byte to the MMC (asmfunc.S) */
uint8_t rcv_spi (void);		/* Send a 0xFF to the MMC and get the received byte (asmfunc.S) */
void dly_100us (void);		/* Delay 100 microseconds (asmfunc.S) */


#endif
