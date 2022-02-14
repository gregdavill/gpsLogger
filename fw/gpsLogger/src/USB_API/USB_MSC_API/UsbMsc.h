/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * ======== UsbMsc.h ========
 */
#include <stdint.h>

#ifndef _USB_MSC_H_
#define _USB_MSC_H_

#include "UsbMscScsi.h"

#ifdef __cplusplus
extern "C"
{
#endif


/*----------------------------------------------------------------------------
 * The following function names and macro names are deprecated.  These were 
 * updated to new names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/
#ifndef DEPRECATED
#define kUSBMSC_idle                        USBMSC_IDLE
#define kUSBMSC_readInProgress              USBMSC_READ_IN_PROGRESS
#define kUSBMSC_writeInProgress             USBMSC_WRITE_IN_PROGRESS
#define kUSBMSC_cmdBeingProcessed           USBMSC_COMMAND_BEING_PROCESSED
#define kUSBMSC_okToSleep                   USBMSC_OK_TO_SLEEP
#define kUSBMSC_processBuffer               USBMSC_PROCESS_BUFFER

#define USBMSC_bufferProcessed              USBMSC_processBuffer
#define USBMSC_updateMediaInfo              USBMSC_updateMediaInformation
#define USBMSC_registerBufInfo              USBMSC_registerBufferInformation
#define USBMSC_poll                         USBMSC_pollCommand
#endif



/*Return values of getState() and USBMSC_pollCommand() API */
#define USBMSC_IDLE               0
#define USBMSC_READ_IN_PROGRESS     1
#define USBMSC_WRITE_IN_PROGRESS    2
#define USBMSC_COMMAND_BEING_PROCESSED  3
#define USBMSC_OK_TO_SLEEP          4
#define USBMSC_PROCESS_BUFFER      5


/*----------------------------------------------------------------------------+
 | Function Prototypes                                                         |
 +----------------------------------------------------------------------------*/

//*****************************************************************************
//
//! \addtogroup msc_state_machine_api
//! @{
//
//******************************************************************************

//*****************************************************************************
//
//! Checks to See if a SCSI Command has Been Received.
//!
//! Checks to see if a SCSI command has been received. If so, it handles it. If not, it returns
//! having taken no action.
//! The return values of this function are intended to be used with entry of low-power modes. If the
//! function returns \b USBMSC_OK_TO_SLEEP, then no further application action is required; that is,
//! either no SCSI command was received; one was received but immediately handled; or one was
//! received but the handling will be completed in the background by the API as it automatically
//! services USB interrupts.
//! If instead the function returns \b USBMSC_PROCESS_BUFFER, then the API is currently servicing a
//! SCSI READ or WRITE command, and the API requires the application to process a buffer. (See
//! Sec. 8.3.6 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a discussion of buffer
//! processing.)
//! Note that even if the function returns these values, the values could potentially be outdated by
//! the time the application evaluates them. For this reason, it's important to disable interrupts prior
//! to calling this function. See Sec. 8.3.5 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC"
//! for more information.
//!
//! \return \b USBMSC_OK_TO_SLEEP or \b USBMSC_PROCESS_BUFFER
//
//*****************************************************************************
uint8_t USBMSC_pollCommand(void);

/* MSC functions */
int16_t MSCToHostFromBuffer ();
int16_t MSCFromHostToBuffer ();
//*****************************************************************************
//
//! This function should be called by the application after it has processed a buffer request.
//!
//! \param USBMSC_Rwbuf_Info*RWBufInfo Pass the value received from USBMSC_fetchInformationStructure().
//!
//! This function should be called by the application after it has processed a buffer request. It
//! indicates to the API that the application has fulfilled the request.
//! Prior to calling this function, the application needs to write a return code to rwInfo.returnCode.
//! This code should reflect the result of the operation. The value may come from the file system
//! software, depending on the application. See Sec. 8.3.6 of
//! \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a list of valid return codes.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBMSC_processBuffer(void);
uint8_t USBMSC_getState ();
//*****************************************************************************
//
//! Informs the API of the Current State of the Media on LUN \b lun.
//!
//! \param lun is the logical unit (LUN) on which the operation is taking place. Zero-based. (This version of the API
//! 	only supports a single LUN.)
//! \param info is a structure that communicates the most recent information about the medium.
//!
//! Informs the API of the current state of the media on LUN \b lun. It does this using an instance \b info
//! of the API-defined structure USBMSC_mediaInfoStr. The API uses the information in the most
//! recent call to this function in automatically handling certain requests from the host.
//! In LUNs that are marked as not removable in USBMSC_CONFIG, this function should be called
//! once at the beginning of execution, prior to attachment to the USB host. It then no longer needs
//! to be called.
//! 
//! In LUNS that are marked as removable, the media information is dynamic. The function should
//! still be called at the beginning of execution to indicate the initial state of the media, and then it
//! should also be called every time the media changes.
//! 
//! See Sec. 8.3.4 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for more about informing
//! the API of media changes.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBMSC_updateMediaInformation (uint8_t lun, struct USBMSC_mediaInfoStr *info);

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************
//*****************************************************************************
//
//! \addtogroup event_handling_api
//! @{
//
//******************************************************************************

//*****************************************************************************
//
//! API Requests a Buffer
//!
//! This event occurs when the API requests a buffer. Immediately prior to this,
//! the API sets the operation field of the USBMSC_RWBuf_Info structure
//! corresponding with the request, and also clears the low-power-mode bits of
//! the MCU's status register to ensure the CPU remains awake to process the
//! buffer after the event occurs. 
//! 
//! NOTE: This means the return value of this event has no effect; the CPU will 															<-- BECAUSE OF THIS...
//! remain awake even if this function returns FALSE.
//!
//! \return FALSE to go to sleep after interrupt
//
//*****************************************************************************
uint8_t USBMSC_handleBufferEvent(void);

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

//*****************************************************************************
//
//! \addtogroup msc_state_machine_api
//! @{
//
//******************************************************************************
//*****************************************************************************
//
//! Gives the API a Buffer to Use for READ/WRITE Data Transfer.
//!
//! \param lun is the Lun number.
//! \param *RWbuf_x is the address of an X-buffer. If null, then both buffers are de-activated.
//! \param *RWbuf_y is the address of an Y-buffer. (Double-buffering is not supported in this version of the API.)
//! \param size is the size, in bytes, of the buffers.
//!
//! Gives the API a buffer to use for READ/WRITE data transfer. \b size indicates the size of the
//! buffer, in bytes.
//! 
//! \b NOTE: Currently, only single-buffering is supported, so \b RWbuf_y should be set to null.
//! If the application intends to allocate the buffer statically, then this function needs only to be
//! called once, prior to any READ/WRITE commands being received from the host. Most likely this
//! would happen during the application's initialization functions.
//! 
//! \b NOTE: This API has to be called after the call to USBMSC_updateMediaInformation() at the beginning
//! of execution.
//! 
//! However, this function optionally enables dynamic buffer management. That is, it can activate
//! and de-activate the buffer, by alternately assigning a null and valid address in \b RWbuf_x. This is
//! useful because the buffer uses a significant portion of the RAM resources (typically 512 bytes).
//! This memory is not needed when USB is not attached or suspended.
//! 
//! If doing this, it's important that the application re-activate the buffer when USB becomes active
//! again, by issuing another call to the function, this time using valid buffer information. If the API
//! needs the buffer and doesn't have it, it will begin failing READ/WRITE commands from the host.
//! The re-activation can take place within USB_handleVbusOffEvent().
//! 
//! \b size must be a multiple of a block size - for FAT, a block size is typically 512 bytes. Thus
//! values of 512, 1024, 1536, etc. are valid. Non-multiples are not valid.
//! 
//! The function returns \b USB_SUCCEED every time. It is up to the application to ensure that the
//! buffers are valid.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBMSC_registerBufferInformation ( uint8_t lun, uint8_t* RWbuf_x, uint8_t* RWbuf_y, uint16_t size);

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************
#ifdef __cplusplus
}
#endif
#endif  //_USB_MSC_H_
//Released_Version_5_20_06_03
