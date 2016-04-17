/* --COPYRIGHT--,NULL
 **
 * --/COPYRIGHT--*/
/*------------------------------------------------------------------------/
 * /  Bitbanging MMCv3/SDv1/SDv2 (in SPI mode) control module
 * /-------------------------------------------------------------------------/
 * /
 * /  Copyright (C) 2010, ChaN, all right reserved.
 * /
 * / * This software is a free software and there is NO WARRANTY.
 * / * No restriction on use. You can use, modify and redistribute it for
 * /   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 * / * Redistributions of source code must retain the above copyright notice.
 * /
 * ---------------------------------------------------------------------------*/
#include <msp430.h>
#include "diskio.h"                                                     /* Common include file for FatFs and disk I/O layer */
#include "HAL_SDCard.h"                                                 /* MSP-EXP430F5529 specific SD Card driver */
#include "main.h"                                                       /* app specific defines */

/*
 *-------------------------------------------------------------------------
 * Platform dependent macros and functions needed to be modified
 *-------------------------------------------------------------------------
 */

#define INIT_PORT()     SDCard_init()                                   /* Initialize MMC control port */
#define FAST_MODE()     SDCard_fastMode()                               /* Maximize SD Card transfer speed */
//#define DLY_US(n)       __delay_cycles(n * 25)  /* Delay n microseconds assuming MCLK=25MHz */
#define DLY_US(n)       __delay_cycles(n * (MCLK_FREQUENCY / 1000000))   //Delay n microseconds           // KLQ


#define CS_H()          SDCard_setCSHigh()                              /* Set MMC CS "high" */
#define CS_L()          SDCard_setCSLow()                               /* Set MMC CS "low" */

//#define	INS             (1)                 /* Card is inserted (yes:true, no:false, default:true) */
uint8_t INS = 1;                                                           //KLQ
#define WP              (0)                                             /* Card is write protected (yes:true, no:false,
                                                                         *default:false) */



/*
 *-------------------------------------------------------------------------
 * Platform dependent RTC Function for FatFs module
 *-------------------------------------------------------------------------
 */

#ifdef __IAR_SYSTEMS_ICC__
#pragma diag_suppress=Pe061
#endif
uint32_t get_fattime (void)
{
    uint32_t tmr;

    //TODO: Customize to use the MSP430 RTC

    /* Pack date and time into a uint32_t variable */
    tmr =     (((uint32_t)2001 - 80) << 25)                                //rtcYear
          | ((uint32_t)9 << 21)                                            //rtcMon
          | ((uint32_t)11 << 16)                                           //rtcMday
          | (uint16_t)(4 << 11)                                             //rtcHour
          | (uint16_t)(30 << 5)                                             //rtcMin
          | (uint16_t)(0 >> 1);                                             //rtcSec

    return (tmr);
}

/*--------------------------------------------------------------------------
 * Module Private Functions
 * ---------------------------------------------------------------------------*/

/* MMC/SD command (SPI mode) */
#define CMD0    (0)                                                     /* GO_IDLE_STATE */
#define CMD1    (1)                                                     /* SEND_OP_COND */
#define ACMD41  (0x80 + 41)                                             /* SEND_OP_COND (SDC) */
#define CMD8    (8)                                                     /* SEND_IF_COND */
#define CMD9    (9)                                                     /* SEND_CSD */
#define CMD10   (10)                                                    /* SEND_CID */
#define CMD12   (12)                                                    /* STOP_TRANSMISSION */
#define ACMD13  (0x80 + 13)                                             /* SD_STATUS (SDC) */
#define CMD16   (16)                                                    /* SET_BLOCKLEN */
#define CMD17   (17)                                                    /* READ_SINGLE_BLOCK */
#define CMD18   (18)                                                    /* READ_MULTIPLE_BLOCK */
#define CMD23   (23)                                                    /* SET_BLOCK_COUNT */
#define ACMD23  (0x80 + 23)                                             /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (24)                                                    /* WRITE_BLOCK */
#define CMD25   (25)                                                    /* WRITE_MULTIPLE_BLOCK */
#define CMD41   (41)                                                    /* SEND_OP_COND (ACMD) */
#define CMD55   (55)                                                    /* APP_CMD */
#define CMD58   (58)                                                    /* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC      0x01                                                /* MMC ver 3 */
#define CT_SD1      0x02                                                /* SD ver 1 */
#define CT_SD2      0x04                                                /* SD ver 2 */
#define CT_SDC      (CT_SD1 | CT_SD2)                                   /* SD */
#define CT_BLOCK    0x08                                                /* Block addressing */


static
DSTATUS Stat = STA_NOINIT;                                              /* Disk status */

static
uint8_t CardType;                                                          /* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */



/* Single byte SPI transfer */

static
BYTE xchg_spi (BYTE dat)
{
	UCA1TXBUF = dat;
	while (UCA1STAT & UCBUSY) ;
	return UCA1RXBUF;
}


/*
 *-----------------------------------------------------------------------
 * Transmit bytes to the MMC
 *-----------------------------------------------------------------------
 */

static
void xmit_mmc (
    const uint8_t* buff,                                                   /* Data to be sent */
    uint16_t bc                                                             /* Number of bytes to send */
    )
{

	if( (bc & 3) != 0)
	{
		while(bc--)
		xchg_spi(*buff++);
		return;
	}


	do{
		//Wait while not ready for TX
		UCA1TXBUF = *buff++;
		while (UCA1STAT & UCBUSY) ;
		//UCA1RXBUF;
		//Wait while not ready for TX
		UCA1TXBUF = *buff++;    					//Write byte
		while (UCA1STAT & UCBUSY) ;
		//UCA1RXBUF;

		//Wait while not ready for TX
		UCA1TXBUF = *buff++;
		while (UCA1STAT & UCBUSY) ;
		//UCA1RXBUF;
		//Wait while not ready for TX
		UCA1TXBUF = *buff++;    					//Write byte
		while (UCA1STAT & UCBUSY) ;
		//UCA1RXBUF;


	}while( bc -=4 );
	//Wait for all TX/RX to finish



	//   UCA1RXBUF;                                              //Dummy read to empty RX buffer
									//and clear any overrun conditions
	//SDCard_sendFrame((uint8_t *)buff, bc);
}

/*
 *-----------------------------------------------------------------------
 * Receive bytes from the MMC
 *-----------------------------------------------------------------------
 */

static
void rcvr_mmc (
    uint8_t *buff,                                                         /* Pointer to read buffer */
    uint16_t bc                                                             /* Number of bytes to receive */
    )
{
    //SDCard_readFrame(buff, bc);
	if( (bc & 3) != 0)
	{
		while(bc--)
		*buff++ = xchg_spi(0xFF);
		return;
	}


	do{
		//while (!(UCA1IFG & UCTXIFG)) ;                      //Wait while not ready for TX
		UCA1TXBUF = 0xff;                                   //Write dummy byte
		while (!(UCA1IFG & UCRXIFG)) ;                      //Wait for RX buffer (full)
		*buff++ = UCA1RXBUF;


		while (!(UCA1IFG & UCTXIFG)) ;                      //Wait while not ready for TX
		UCA1TXBUF = 0xff;                                   //Write dummy byte
		while (!(UCA1IFG & UCRXIFG)) ;                      //Wait for RX buffer (full)
		*buff++ = UCA1RXBUF;

		//while (!(UCA1IFG & UCTXIFG)) ;                      //Wait while not ready for TX
		UCA1TXBUF = 0xff;                                   //Write dummy byte
		while (!(UCA1IFG & UCRXIFG)) ;                      //Wait for RX buffer (full)
		*buff++ = UCA1RXBUF;


		while (!(UCA1IFG & UCTXIFG)) ;                      //Wait while not ready for TX
		UCA1TXBUF = 0xff;                                   //Write dummy byte
		while (!(UCA1IFG & UCRXIFG)) ;                      //Wait for RX buffer (full)
		*buff++ = UCA1RXBUF;
	}while( bc -=4 );



}

/*
 *-----------------------------------------------------------------------
 * Wait for card ready
 *-----------------------------------------------------------------------
 */

static
int16_t wait_ready (void)                                                   /* 1:OK, 0:Timeout */
{
    uint8_t d;
    uint16_t tmr;


    for (tmr = 5000; tmr; tmr--){                                       /* Wait for ready in timeout of 500ms */
    	d = xchg_spi(0xFF);
        if (d == 0xFF){
            return ( 1) ;
        }
        DLY_US(10);
    }

    return (0);
}

/*
 *-----------------------------------------------------------------------
 * Deselect the card and release SPI bus
 *-----------------------------------------------------------------------
 */

static
void deselect (void)
{
    uint8_t d;

    CS_H();
    xchg_spi(0xFF);
}

/*
 *-----------------------------------------------------------------------
 * Select the card and wait for ready
 *-----------------------------------------------------------------------
 */

static
int16_t select (void)                   /* 1:OK, 0:Timeout */
{
    CS_L();
    if (!wait_ready()){
        deselect();
        return (0);
    }
    return (1);
}

/*
 *-----------------------------------------------------------------------
 * Receive a data packet from MMC
 *-----------------------------------------------------------------------
 */

static
int16_t rcvr_datablock (                /* 1:OK, 0:Failed */
    uint8_t *buff,                     /* Data buffer to store received data */
    uint16_t btr                        /* Byte count */
    )
{
	uint8_t c;
    uint8_t d[2];
    uint16_t tmr;


    for (tmr = 1000; tmr; tmr--){   /* Wait for data packet in timeout of 100ms */
    	c = xchg_spi(0xFF);
        if (c != 0xFF){
            break;
        }
        DLY_US(10);
    }
    if (c != 0xFE){
        return ( 0) ;               /* If not valid data token, retutn with error */
    }
    rcvr_mmc(buff, btr);            /* Receive the data block into buffer */
    xchg_spi(0xFF);                 /* Discard CRC */
    xchg_spi(0xFF);

    return (1);                     /* Return with success */
}

/*
 *-----------------------------------------------------------------------
 * Send a data packet to MMC
 *-----------------------------------------------------------------------
 */

static
int16_t xmit_datablock (                /* 1:OK, 0:Failed */
    const uint8_t *buff,               /* 512 byte data block to be transmitted */
    uint8_t token                      /* Data/Stop token */
    )
{
	uint8_t c;
    uint8_t d[2];


    if (!wait_ready()){
        return ( 0) ;
    }

   // d[0] = token;
    xchg_spi(token);                /* Xmit a token */
    if (token != 0xFD){             /* Is it data token? */
        xmit_mmc(buff, 512);        /* Xmit the 512 byte data block to MMC */
        xchg_spi(0xFF);
        xchg_spi(0xFF);             /* Dummy CRC (FF,FF) */
        c = xchg_spi(0xFF);             /* Receive data response */
        if ((c & 0x1F) != 0x05){ /* If not accepted, return with error */
            return (0);
        }
    }

    return (1);
}

/*
 *-----------------------------------------------------------------------
 * Send a command packet to MMC
 *-----------------------------------------------------------------------
 */

static
uint8_t send_cmd (                     /* Returns command response (bit7==1:Send failed)*/
    uint8_t cmd,                       /* Command byte */
    uint32_t arg                       /* Argument */
    )
{
	BYTE n, res;


		if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
			cmd &= 0x7F;
			res = send_cmd(CMD55, 0);
			if (res > 1) return res;
		}

		/* Select the card and wait for ready except to stop multiple block read */
		if (cmd != CMD12) {
			deselect();
			if (!select()) return 0xFF;
		}

		/* Send command packet */
		xchg_spi(0x40 | cmd);			/* Start + Command index */
		xchg_spi((BYTE)(arg >> 24));	/* Argument[31..24] */
		xchg_spi((BYTE)(arg >> 16));	/* Argument[23..16] */
		xchg_spi((BYTE)(arg >> 8));		/* Argument[15..8] */
		xchg_spi((BYTE)arg);			/* Argument[7..0] */
		n = 0x01;						/* Dummy CRC + Stop */
		if (cmd == CMD0) n = 0x95;		/* Valid CRC for CMD0(0) + Stop */
		if (cmd == CMD8) n = 0x87;		/* Valid CRC for CMD8(0x1AA) + Stop */
		xchg_spi(n);

		/* Receive command response */
		if (cmd == CMD12) xchg_spi(0xFF);	/* Skip a stuff byte on stop to read */
		n = 10;							/* Wait for a valid response in timeout of 10 attempts */
		do
			res = xchg_spi(0xFF);
		while ((res & 0x80) && --n);

		return res;			/* Return with the response value */
}

/*--------------------------------------------------------------------------
 *
 * Public Functions
 *
 * ---------------------------------------------------------------------------*/


/*
 *-----------------------------------------------------------------------
 * Get Disk Status
 *-----------------------------------------------------------------------
 */

DSTATUS disk_status (
    uint8_t drv                        /* Drive number (0) */
    )
{
    DSTATUS s = Stat;


    if (drv || !INS){
        s = STA_NODISK | STA_NOINIT;
    } else {
        s &= ~STA_NODISK;
        if (WP){
            s |= STA_PROTECT;
        } else {
            s &= ~STA_PROTECT;
        }
    }
    Stat = s;

    return (s);
}

/*
 *-----------------------------------------------------------------------
 * Initialize Disk Drive
 *-----------------------------------------------------------------------
 */

DSTATUS disk_initialize (
    uint8_t drv    /* Physical drive nmuber (0) */
    )
{
/* TI: Inserted pragma to supress IAR compiler warning incicating 'cmd'
 * is not used. If removed however the compile fails */
#ifdef __IAR_SYSTEMS_ICC__
#pragma diag_suppress=Pe550
#endif
#ifdef __TI_COMPILER_VERSION__
#pragma diag_suppress 552
#endif
    uint8_t n, ty, cmd, ocr[4];
#ifdef __IAR_SYSTEMS_ICC__
#pragma diag_default=Pe550
#endif
//#ifdef __TI_COMPILER_VERSION__
//#pragma diag_default 552
//#endif
    uint16_t tmr;
    DSTATUS s;


    INIT_PORT();                                                        /* Initialize control port */

    s = disk_status(drv);                                               /* Check if card is in the socket */
    if (s & STA_NODISK){
        return ( s) ;
    }

    CS_H();
    for (n = 10; n; n--) xchg_spi(0xff);                              /* 80 dummy clocks */

    ty = 0;
    if (send_cmd(CMD0, 0) == 1){                                        /* Enter Idle state */
        if (send_cmd(CMD8, 0x1AA) == 1){                                /* SDv2? */
        	for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);			/* Get trailing return value of R7 resp */
        	if (ocr[2] == 0x01 && ocr[3] == 0xAA) {                     /* The card can work at vdd range of 2.7-3.6V */
                for (tmr = 1000; tmr; tmr--)
                {                                                       /* Wait for leaving idle state (ACMD41 with HCS bit) */
                    if (send_cmd(ACMD41, 0x40000000) == 0){
                        break;
                    }
                    DLY_US(1000);
                }
                if (tmr && send_cmd(CMD58, 0) == 0){                    /* Check CCS bit in the OCR */
                	for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
                	ty = (ocr[0] & 0x40) ? CT_SD2|CT_BLOCK : CT_SD2;	/* SDv2 */
                }
            }
        } else {                                                        /* SDv1 or MMCv3 */
            if (send_cmd(ACMD41, 0) <= 1){
                ty = CT_SD1; cmd = ACMD41;                              /* SDv1 */
            } else {
                ty = CT_MMC; cmd = CMD1;                                /* MMCv3 */
            }
            for (tmr = 1000; tmr; tmr--)
            {                                                           /* Wait for leaving idle state */
                if (send_cmd(ACMD41, 0) == 0){
                    break;
                }
                DLY_US(1000);
            }
            if (!tmr || send_cmd(CMD16, 512) != 0){                     /* Set R/W block length to 512 */
                ty = 0;
            }
        }
    }
    CardType = ty;
    deselect();

    if (ty){                                                            /* Initialization succeded */
        FAST_MODE();
        s &= ~STA_NOINIT;
    } else {                                                            /* Initialization failed */
        s |= STA_NOINIT;
    }

    Stat = s;

    return (s);
}

/*
 *-----------------------------------------------------------------------
 * Read Sector(s)
 *-----------------------------------------------------------------------
 */

DRESULT disk_read (
    uint8_t drv,                                                           /* Physical drive nmuber (0) */
    uint8_t *buff,                                                         /* Pointer to the data buffer to store read data */
    uint32_t sector,                                                       /* Start sector number (LBA) */
	UINT count                                                          /* Sector count (1..128) */
    )
{
    DSTATUS s;


    s = disk_status(drv);
    if (s & STA_NOINIT){
        return ( RES_NOTRDY) ;
    }
    if (!count){
        return ( RES_PARERR) ;
    }
    if (!(CardType & CT_BLOCK)){
        sector *= 512;                                                  /* Convert LBA to byte address if needed */
    }
    if (count == 1){                                                    /* Single block read */
        if ((send_cmd(CMD17, sector) == 0)                              /* READ_SINGLE_BLOCK */
            && rcvr_datablock(buff, 512)){
            count = 0;
        }
    } else {                                                            /* Multiple block read */
        if (send_cmd(CMD18, sector) == 0){                              /* READ_MULTIPLE_BLOCK */
            do {
                if (!rcvr_datablock(buff, 512)){
                    break;
                }
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);                                         /* STOP_TRANSMISSION */
        }
    }
    deselect();

    return (count ? RES_ERROR : RES_OK);
}

/*
 *-----------------------------------------------------------------------
 * Write Sector(s)
 *-----------------------------------------------------------------------
 */

DRESULT disk_write (
    uint8_t drv,                                                           /* Physical drive nmuber (0) */
    const uint8_t *buff,                                                   /* Pointer to the data to be written */
    uint32_t sector,                                                       /* Start sector number (LBA) */
	UINT count                                                          /* Sector count (1..128) */
    )
{
    DSTATUS s;


    s = disk_status(drv);
    if (s & STA_NOINIT){
        return ( RES_NOTRDY) ;
    }
    if (s & STA_PROTECT){
        return ( RES_WRPRT) ;
    }
    if (!count){
        return ( RES_PARERR) ;
    }
    if (!(CardType & CT_BLOCK)){
        sector *= 512;                                                  /* Convert LBA to byte address if needed */
    }
    if (count == 1){                                                    /* Single block write */
        if ((send_cmd(CMD24, sector) == 0)                              /* WRITE_BLOCK */
            && xmit_datablock(buff, 0xFE)){
            count = 0;
        }
    } else {                                                            /* Multiple block write */
        if (CardType & CT_SDC){
            send_cmd(ACMD23, count);
        }
        if (send_cmd(CMD25, sector) == 0){                              /* WRITE_MULTIPLE_BLOCK */
            do {
                if (!xmit_datablock(buff, 0xFC)){
                    break;
                }
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD)){                              /* STOP_TRAN token */
                count = 1;
            }
        }
    }
    deselect();

    return (count ? RES_ERROR : RES_OK);
}

/*
 *-----------------------------------------------------------------------
 * Miscellaneous Functions
 *-----------------------------------------------------------------------
 */

DRESULT disk_ioctl (
    uint8_t drv,                                                           /* Physical drive nmuber (0) */
    uint8_t ctrl,                                                          /* Control code */
    void* buff /* Buffer to send/receive control data */               //KLQ  Changed from void* to uint32_t*
    )
{
    DRESULT res;
    uint8_t n, csd[16];
    uint16_t cs;


    if (disk_status(drv) & STA_NOINIT){                                 /* Check if card is in the socket */
        return (RES_NOTRDY);
    }

    res = RES_ERROR;
    switch (ctrl){
        case CTRL_SYNC:                                                 /* Make sure that no pending write process */
            if (select()){
                deselect();
                res = RES_OK;
            }
            break;

        case GET_SECTOR_COUNT:                                          /* Get number of sectors on the disk (uint32_t) */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)){
                if ((csd[0] >> 6) == 1){                                /* SDC ver 2.00 */
                    cs = csd[9] + ((uint16_t)csd[8] << 8) + 1;
                    *(uint32_t*)buff = ((uint32_t)cs) << 10;
                } else {                                                /* SDC ver 1.XX or MMC */
                    n =
                        (csd[5] &
                         15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    cs =
                        (csd[8] >>
                         6) +
                        ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
                    *(uint32_t*)buff = (uint32_t)cs << (n - 9);
                }
                res = RES_OK;
            }
            break;

        case GET_BLOCK_SIZE:                                            /* Get erase block size in unit of sector (uint32_t) */
            *(uint32_t*)buff = 128;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
    }

    deselect();

    return (res);
}

//KLQ

uint8_t validateCSD (void)
{
    uint8_t csd0[16], csd1[16], i;
    uint16_t sum = 0;

    //Pull the CSD -- twice.  If the response codes are invalid, then we know the card isn't there or initialized.
    if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd0, 16)){
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd1, 16)){
            //The response codes were good -- but maybe the SPI input was just floating low.  Let's evaluate the CSD data.
            //First, look for all zero or all ones.  If the SPI input is floating, these are the most likely outcomes.
            for (i = 0; i <= 15; i++){
                sum += csd0[i];
            }
            if (!((sum == 0) || (sum == 4096))){
                //The response was a mix of 0's and 1's.  Floating inputs could still do that -- but it's unlikely they'd
                //produce the same pattern twice.  Compare to ensure the two are identical.
                i = 0;
                while (i <= 15)
                {
                    if (csd0[i] != csd1[i]){
                        break;
                    }
                    i++;
                }
                if (i > 15){
                    return ( 1) ;
                }
            }
        }
    }
    return ( 0) ;
}

//Attempt to detect the card by commanding it to return its CSD register and evaluating it.  Returns the
//result, and also updates FatFs's internal INS variable.
//The proper way to detect a card is by sensing its presence on the DAT3 signal.  The EXP board doesn't
//contain the necessary h/w, so this s/w method works instead.
uint8_t detectCard (void)
{
    //Check for a valid CSD response
    if (validateCSD()){
        disk_status(0);     //Update the INS variable
        return ( 1) ;       //Card is present
    }

    //We didn't get a valid response.  So we now know the status is one of two things:
    //a) The card isn't there at all;
    //b) or, it was just inserted recently, and needs to be initialized

    INS = 0x01;             //Trick disk_initialize into thinking it's inserted...
    disk_initialize(0);     //Attempt to initialize it

    INS = validateCSD();    //Try again
    disk_status(0);         //Update the INS variable

    return ( INS) ;         //1 = card is present; 0 = not present
}

//Released_Version_5_00_01
