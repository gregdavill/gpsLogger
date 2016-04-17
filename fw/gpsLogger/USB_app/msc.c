/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*
 * ======== msc.c ========
 *
 *
 */
#include <string.h>
#include <stdint.h>
#include "driverlib.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_MSC_API/UsbMsc.h"
#include "USB_config/descriptors.h"

#include "FatFs/diskio.h"
#include "FatFs/mmc.h"
#include "USB_app/msc.h"

#include "hal.h"
const uint16_t BYTES_PER_BLOCK = 512;

/*
 * Every application using this MSC API must define an instance of this
 * structure.  It establishes compile-time information about the storage media.
 */
extern struct config_struct USBMSC_config;


//Data-exchange buffer between the API and the application.
//The application allocates it, and then registers it with the API.
//Later, the API will pass it back to the application when it
//needs the application to "process" it (exchange data with the media).
// In this case each cluster has one block so this buffer is 512 bytes.
uint8_t RWbuf[512];
uint8_t RWbufA[512];

uint8_t toggle = 0;

//The API allocates an instance of structure type USBMSC_RWbuf_Info to hold all
//information describing buffers to be processed.  The structure instance is a
//shared resource between the API and application.
//During initialization, we'll call USBMSC_fetchInformationStructure() to obtain the
//pointer from the API.
USBMSC_RWbuf_Info *RWbuf_info;


//The application must tell the API about the media.  This information is
//conveyed in a call to USBMSC_updateMediaInformation(), passing an instance of
//USBMSC_RWbuf_Info.

struct USBMSC_mediaInfoStr mediaInfo;

//Global flag by which the timer ISR will trigger main() to check the
//media status
uint8_t bDetectCard = 0x00;

/*
 * This function initializes the MSC data variables
 */
void USBMSC_initMSC(void)
{
    //SD-cards must go through a setup sequence after powerup.
    //This FatFs call does this.
    disk_initialize(0);

    //The API maintains an instance of the USBMSC_RWbuf_Info structure.This is
    // a shared resource between the API and application; the application must
    //request the pointer.

    RWbuf_info = USBMSC_fetchInformationStructure();


    //USBMSC_updateMediaInformation() must be called prior to USB connection.  We
    //check if the card is present, and if so, pull its size and bytes per
    //block.
    if (detectCard()){
        mediaInfo.mediaPresent = USBMSC_MEDIA_PRESENT;
    }
	else {
	    mediaInfo.mediaPresent = USBMSC_MEDIA_NOT_PRESENT;
	}
    mediaInfo.mediaChanged = 0x00;
    mediaInfo.writeProtected = 0x00;
    //Returns the number of blocks (sectors) in the media.
    disk_ioctl(0,GET_SECTOR_COUNT,&mediaInfo.lastBlockLba);
    //Block size will always be 512
    mediaInfo.bytesPerBlock = BYTES_PER_BLOCK;
    USBMSC_updateMediaInformation(0, &mediaInfo);

    //The data interchange buffer (used when handling SCSI READ/WRITE) is
    //declared by the application, and registered with the API using this
    //function.  This allows it to be assigned dynamically, giving
    //the application more control over memory management.
    USBMSC_registerBufferInformation(0, &RWbuf[0], NULL, sizeof(RWbuf));
}

void USBMSC_processMSCBuffer(void)
{
    //Call USBMSC_pollCommand() to initiate handling of any received SCSI commands.
    //Disable interrupts
    //during this function, to avoid conflicts arising from SCSI commands
    //being received from the host
    //AFTER decision to enter LPM is made, but BEFORE it's actually entered
    //(in other words, avoid
    //sleeping accidentally).

	__disable_interrupt();
	if ((USBMSC_pollCommand() == USBMSC_OK_TO_SLEEP) && (!bDetectCard)){
	     //Enable interrupts atomically with LPM0 entry
		__bis_SR_register(LPM0_bits + GIE);
	}
	__enable_interrupt();


	//Verify SD card before attempting to read.  If this condition is not there Windows displays
	//a message about 'Reformatting the disk' when SD card is not in the drive.
	if(detectCard()){   //Bug number 6754
		bDetectCard = 0x01;  //update the flag.  There seems to be a timing issue between when the main 
                             //application calls USBMSC_processMSCBuffer() and when the card is detected.  Setting
                             //the global flag here prevents the Windows message 'Reformat Disk' from 
                             //showing up when an SD card is inserted into an empty drive.

		//If the API needs the application to process a buffer, it will keep the
		//CPU awake by returning USBMSC_PROCESS_BUFFER
		//from USBMSC_pollCommand().  The application should then check the 'operation'
		//field of all defined USBMSC_RWbuf_Info
		//structure instances.  If any of them is non-null, then an operation
		//needs to be processed.  A value of
		//USBMSC_READ indicates the API is waiting for the application to fetch
		//data from the storage volume, in response
		//to a SCSI READ command from the USB host.  After the application does
		//this, it must indicate whether the
		//operation succeeded, and then close the buffer operation by calling
		//USBMSC_processBuffer().

		while (RWbuf_info->operation == USBMSC_READ)
		{
			hal_led_a(GREEN);
			//A READ operation is underway, and the app has been requested to access
			//the medium.  So, call file system to read
			//to do so.  Note this is a low level FatFs call -- we are not
			//attempting to open a file ourselves.  The host is
			//in control of this access, we're just carrying it out.
			DRESULT dresult = disk_read(0, //Physical drive number (0)
				RWbuf_info->bufferAddr,    //Pointer to the user buffer
				RWbuf_info->lba,           //First LBA of this buffer operation
				RWbuf_info->lbCount);      //The number of blocks being requested
											//as part of this operation
			hal_led_a(0);

			//The result of the file system call needs to be communicated to the
			//host.  Different file system software uses
			//different return codes, but they all communicate the same types of
			//results.  This code ultimately gets passed to the
			//host app that issued the command to read (or if the user did it the
			//host OS, perhaps in a dialog box).
			switch (dresult)
			{
				case RES_OK:
					RWbuf_info->returnCode = USBMSC_RW_SUCCESS;
					break;
				//In FatFs, this result suggests the medium may have been removed
				//recently.
				case RES_ERROR:
					//This application function checks for the SD-card,
					//and if missing,calls USBMSC_updateMediaInformation() to inform
					//the API
					if (!USBMSC_checkMSCInsertionRemoval()){
						RWbuf_info->returnCode =
							USBMSC_RW_MEDIA_NOT_PRESENT;
					}
					break;
				case RES_NOTRDY:
					RWbuf_info->returnCode = USBMSC_RW_NOT_READY;
					break;
				case RES_PARERR:
					RWbuf_info->returnCode = USBMSC_RW_LBA_OUT_OF_RANGE;
					break;
			}
			hal_led_a(RED);
			USBMSC_processBuffer();
			hal_led_a(0);
		}

		//Everything in this section is analogous to READs.  Reference the
		//comments above.
		while (RWbuf_info->operation == USBMSC_WRITE)
		{
			hal_led_a(GREEN);
			DRESULT dresult = disk_write(0, //Physical drive number (0)
				RWbuf_info->bufferAddr,    //Pointer to the user buffer
				RWbuf_info->lba,           //First LBA of this buffer operation
				RWbuf_info->lbCount);      //The number of blocks being requested
											//as part of this operation
			hal_led_a(0);

			switch (dresult)
			{
				case RES_OK:
					RWbuf_info->returnCode = USBMSC_RW_SUCCESS;
					break;
				case RES_ERROR:
					if (!USBMSC_checkMSCInsertionRemoval()){
						RWbuf_info->returnCode =
							USBMSC_RW_MEDIA_NOT_PRESENT;
					}
					break;
				case RES_NOTRDY:
					RWbuf_info->returnCode = USBMSC_RW_NOT_READY;
					break;
				case RES_PARERR:
					RWbuf_info->returnCode = USBMSC_RW_LBA_OUT_OF_RANGE;
					break;
				default:
					RWbuf_info->returnCode = USBMSC_RW_NOT_READY;
					break;
			}
			hal_led_a(RED);
			USBMSC_processBuffer();
			hal_led_a(0);
		}
	}
}

/*
 * ======== USBMSC_checkMSCInsertionRemoval ========
 *
 * This function checks for insertion/removal of the card.  If either is
 * detected, it informs the API by calling USBMSC_updateMediaInformation().  Whether
 * it detects it or not, it returns non-zero if the card is present, or zero if
 * not present
 */
uint8_t USBMSC_checkMSCInsertionRemoval (void)
{
    //Check card status -- there or not?
    uint8_t newCardStatus = detectCard();

    if ((newCardStatus) &&
        (mediaInfo.mediaPresent == USBMSC_MEDIA_NOT_PRESENT)){
        //An insertion has been detected -- inform the API
        mediaInfo.mediaPresent = USBMSC_MEDIA_PRESENT;
        mediaInfo.mediaChanged = 0x01;
        //Get the size of this new medium
        DRESULT SDCard_result = disk_ioctl(0,
            GET_SECTOR_COUNT,
            &mediaInfo.lastBlockLba);
        USBMSC_updateMediaInformation(0, &mediaInfo);
    }

    if ((!newCardStatus) && (mediaInfo.mediaPresent == USBMSC_MEDIA_PRESENT)){
        //A removal has been detected -- inform the API
        mediaInfo.mediaPresent = USBMSC_MEDIA_NOT_PRESENT;
        mediaInfo.mediaChanged = 0x01;
        USBMSC_updateMediaInformation(0, &mediaInfo);
    }

    return ( newCardStatus) ;
}
//Released_Version_5_00_01
