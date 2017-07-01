#include "msp430.h"
#include "mmc.h"
#include "hal.h"



//Pins from MSP430 connected to the SD Card
#define SPI_SIMO        BIT4 // P4
#define SPI_SOMI        BIT5 // P4
#define SPI_CLK         BIT0 // P4
#define SD_CS           BIT3 // P4

//Ports
#define SPI_SEL         P4SEL
#define SPI_DIR         P4DIR
#define SD_CS_SEL       P4SEL
#define SD_CS_OUT       P4OUT
#define SD_CS_DIR       P4DIR

//KLQ
#define SPI_REN         P4REN
#define SPI_OUT         P4OUT
//KLQ


static int16_t wait_ready (void) ;

/***************************************************************************//**
 * @brief   deInitialize SD Card
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_deinit (void)
{
    //Port initialization for SD Card operation
    SPI_SEL &= ~(SPI_SOMI | SPI_SIMO | SPI_CLK);
    SPI_DIR &= ~(SPI_SIMO | SPI_CLK); // output low
    SD_CS_SEL &= ~SD_CS;
    SD_CS_OUT &= ~SD_CS; // output low
    SD_CS_DIR |= SD_CS;

    //KLQ
    SPI_REN |= SPI_SOMI | SPI_SIMO;
    SPI_OUT &= ~(SPI_SOMI | SPI_SIMO); // pull down on both? data lines
    //KLQ


    /* Enable power to SD card */
   // hal_sd_pwr_on();

    //Initialize USCI_B1 for SPI Master operation
    UCA1CTL1 |= UCSWRST;                                    //Put state machine in reset
    UCA1CTL0 = 0;  //3-pin, 8-bit SPI master

}

/***************************************************************************//**
 * @brief   Enable fast SD Card SPI transfers. This function is typically
 *          called after the initial SD Card setup is done to maximize
 *          transfer speed.
 * @param   None
 * @return  None
 ******************************************************************************/
void SDCard_fastMode (void)
{
    UCA1CTL1 |= UCSWRST;                                    //Put state machine in reset
    UCA1BR0 = 10;                                            //f_UCxCLK = 25MHz/2 = 12.5MHz
    UCA1BR1 = 0;
    UCA1CTL1 &= ~UCSWRST;                                   //Release USCI state machine
}


void init_spi (void)
{
	  //Port initialization for SD Card operation
	    SPI_SEL |= SPI_SOMI | SPI_SIMO | SPI_CLK;
	    SPI_DIR |= SPI_SIMO | SPI_CLK;
	    SD_CS_SEL &= ~SD_CS;
	    SD_CS_OUT |= SD_CS;
	    SD_CS_DIR |= SD_CS;

	    //KLQ
	    SPI_REN |= SPI_SOMI | SPI_SIMO;
	    SPI_OUT |= SPI_SOMI | SPI_SIMO;
	    //KLQ


	    /* Enable power to SD card */
	   // hal_sd_pwr_on();

	    //Initialize USCI_B1 for SPI Master operation
	    UCA1CTL1 |= UCSWRST;                                    //Put state machine in reset
	    UCA1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;  //3-pin, 8-bit SPI master
	    //Clock polarity select - The inactive state is high
	    //MSB first
	    UCA1CTL1 = UCSWRST | UCSSEL__SMCLK;                          //Use SMCLK, keep RESET
	    UCA1BR0 = 63;                                           //Initial SPI clock must be <400kHz
	    UCA1BR1 = 0;                                            //f_UCxCLK = 25MHz/63 = 397kHz
	    UCA1CTL1 &= ~UCSWRST;                                   //Release USCI state machine
	    UCA1IFG &= ~UCRXIFG;
}


void deselect (void)
{
	SD_CS_OUT |= SD_CS;
	xmit_spi(0xFF);
}
void select (void)
{
    SD_CS_OUT &= ~SD_CS;

    if (!wait_ready()){
     }
}

void xmit_spi (uint8_t d)
{
	UCA1TXBUF = (uint8_t)d;
	while (UCA1STAT & UCBUSY) ;

}


uint8_t rcv_spi (void)
{
	UCA1TXBUF = 0xFF;
	while (UCA1STAT & UCBUSY) ;
	return UCA1RXBUF;
}


void dly_100us (void) {
	 __delay_cycles(100 * (MCLK_FREQUENCY / 1000000));
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


    for (tmr = 500; tmr; tmr--){                                       /* Wait for ready in timeout of 500ms */
    	d = rcv_spi();
        if (d == 0xFF){
            return ( 1) ;
        }
        dly_100us();
    }

    return (0);
}

