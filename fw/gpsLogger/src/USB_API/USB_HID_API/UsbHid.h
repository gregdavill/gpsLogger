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
 * ======== UsbHid.h ========
 */
#include <stdint.h>

#ifndef _UsbHid_H_
#define _UsbHid_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------
 * The following function names and macro names are deprecated.  These were 
 * updated to new names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/
#ifndef DEPRECATED
#define  kUSBHID_sendStarted          USBHID_SEND_STARTED
#define  kUSBHID_sendComplete         USBHID_SEND_COMPLETE
#define  kUSBHID_intfBusyError        USBHID_INTERFACE_BUSY_ERROR
#define  kUSBHID_receiveStarted       USBHID_RECEIVE_STARTED
#define  kUSBHID_receiveCompleted     USBHID_RECEIVE_COMPLETED
#define  kUSBHID_receiveInProgress    USBHID_RECEIVE_IN_PROGRESS
#define  kUSBHID_generalError         USBHID_GENERAL_ERROR
#define  kUSBHID_busNotAvailable      USBHID_BUS_NOT_AVAILABLE
#define  kUSBHID_waitingForSend       USBHID_WAITING_FOR_SEND
#define  kUSBHID_waitingForReceive    USBHID_WAITING_FOR_RECEIVE
#define  kUSBHID_dataWaiting          USBHID_DATA_WAITING
#define  kUSB_allHidEvents            USBHID_ALL_HID_EVENTS
#define  kUSBHID_noDataWaiting        USBHID_NO_DATA_WAITING

#define   USBHID_intfStatus           USBHID_getInterfaceStatus
#define   USBHID_bytesInUSBBuffer     USBHID_getBytesInUSBBuffer

#endif



#define USBHID_SEND_STARTED         0x01
#define USBHID_SEND_COMPLETE        0x02
#define USBHID_INTERFACE_BUSY_ERROR       0x03
#define USBHID_RECEIVE_STARTED      0x04
#define USBHID_RECEIVE_COMPLETED    0x05
#define USBHID_RECEIVE_IN_PROGRESS   0x06
#define USBHID_GENERAL_ERROR        0x07
#define USBHID_BUS_NOT_AVAILABLE     0x08
#define HID_BOOT_PROTOCOL       0x00
#define HID_REPORT_PROTOCOL     0x01
//returned by USBHID_rejectData() if no data pending
#define USBHID_NO_DATA_WAITING        1 
#define USBHID_WAITING_FOR_SEND      0x01
#define USBHID_WAITING_FOR_RECEIVE   0x02
#define USBHID_DATA_WAITING         0x04
#define USBHID_BUS_NOT_AVAILABLE     0x08
#define USBHID_ALL_HID_EVENTS           0xFF

#define USBHID_handleGetReport USBHID_handleEP0GetReport
#define USBHID_handleSetReport USBHID_handleEP0SetReport
#define USBHID_handleSetReportDataAvailable USBHID_handleEP0SetReportDataAvailable
#define USBHID_handleSetReportDataAvailable  USBHID_handleEP0SetReportDataAvailable


/*----------------------------------------------------------------------------
 * These functions can be used in application
 +----------------------------------------------------------------------------*/

//*****************************************************************************
//
//! \addtogroup hid_api
//! @{
//
//******************************************************************************

//*****************************************************************************
//
//! Sends a Data Report to the Host.
//!
//! \param reportData is an array containing the report.
//! \param intfNum is which HID interface the data should be transmitted
//! 	over.
//!
//! Sends a pre-built report \b reportData to the host, on interface
//! \b intfNum. The report must be organized to reflect the format defined
//! by the report descriptor in descriptors.c.
//! 
//! When the function returns \b USBHID_SEND_COMPLETE, the data has been written to
//! the USB transmit buffers, and will be transferred to the host in the next
//! polling frame. If the function returns \b USBHID_BUS_NOT_AVAILABLE, then the bus
//! has either been disconnected or the device is suspended, allowing no reports
//! to be sent. If the function returns \b USBHID_INTERFACE_BUSY_ERROR, it means the USB
//! buffer for the interface has data in it, suggesting that the host has not
//! yet fetched the previously-loaded report.
//!
//! \return Any of the following:
//! 		- \b USBHID_SEND_COMPLETE
//! 		- \b USBHID_BUS_NOT_AVAILABLE
//! 		- \b USBHID_INTERFACE_BUSY_ERROR
//
//*****************************************************************************
extern uint8_t USBHID_sendReport (const uint8_t * reportData, uint8_t intfNum);

//*****************************************************************************
//
//! Receives a Report from the Host into \b reportData.
//!
//! \param reportData is an array containing the report.
//! \param intfNum is the HID interface over which the data is to be
//! 	received.
//!
//! Receives a report from the host into \b reportData, on interface
//! \b intfNum. It is expected that the host will organize the report in the
//! format defined by the report descriptor in descriptors.c.
//! 
//! When the function returns \b USBHID_RECEIVE_COMPLETED, the data has been
//! successfully copied from the USB receive buffers into \b reportData. If the
//! function returns \b USBHID_BUS_NOT_AVAILABLE, then the bus has either been
//! disconnected or the device is suspended, allowing no reports to be sent. If
//! the function returns \b USBHID_GENERAL_ERROR, it means the call failed for
//! unspecified reasons.
//! 
//! The call is intended to be called only when it is known that a report is in
//! the USB buffer. This means it is best called in response to the API calling
//! USBHID_handleDataReceived(), which indicates that a report has been received
//! for interface \b intfNum.
//! 
//! \return Any of the following:
//! 		- \b USBHID_RECEIVE_COMPLETED
//! 		- \b USBHID_BUS_NOT_AVAILABLE
//! 		- \b USBHID_INTERFACE_BUSY_ERROR
//
//*****************************************************************************
uint8_t USBHID_receiveReport (uint8_t * reportData, uint8_t intfNum);

//*****************************************************************************
//
//! Initiates Sending of a User Buffer Over HID Interface.
//!
//! \param data is an array of data to be sent.
//! \param size is the number of bytes to be sent, starting from address
//! 	\b data.
//! \param intfNum is which data interface the \b data should be transmitted
//! 	over.
//!
//! Initiates sending of a user buffer over HID interface \b intfNum, of size
//! \b size and starting at address \b data. If \b size is larger than the
//! packet size, the function handles all packetization and buffer management.
//! \b size has no inherent upper limit (beyond being a 16-bit value).
//! 
//! In most cases where a send operation is successfully started, the function
//! will return \b USBHID_SEND_STARTED. A send operation is said to be underway. At
//! some point, either before or after the function returns, the send operation
//! will complete, barring any events that would preclude it. (Even if the
//! operation completes before the function returns, the return code will still
//! be \b USBHID_SEND_STARTED.)
//! 
//! If the bus is not connected when the function is called, the function
//! returns \b USBHID_BUS_NOT_AVAILABLE, and no operation is begun. If \b size is 0,
//! the function returns \b USBHID_GENERAL_ERROR. If a previous send operation is
//! already underway for this data interface, the function returns with
//! \b USBHID_INTERFACE_BUSY_ERROR.
//! 
//! USB includes low-level mechanisms that ensure valid transmission of data.
//! 
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! send operations.
//! 
//! \return Any of the following:
//! 		- \b USBHID_SEND_STARTED: a send operation was successfully
//! 			started.
//! 		- \b USBHID_INTERFACE_BUSY_ERROR: a previous send operation is
//! 			underway.
//! 		- \b USBHID_BUS_NOT_AVAILABLE: the bus is either suspended or
//! 			disconnected.
//! 		- \b USBHID_GENERAL_ERROR: \b size was zero, or other
//! 			errorkUSBHID_receiveCompleted.
//
//*****************************************************************************
uint8_t USBHID_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum);


//*****************************************************************************
//
//! Receives \b size Bytes Over HID Interface.
//!
//! \param data is an array to contain the data received.
//! \param size is the number of bytes to be received.
//! \param intfNum is which data interface to receive from.
//!
//! Receives \b size bytes over HID interface \b intfNum into memory starting at
//! address \b data. \b size has no inherent upper limit (beyond being a 16-bit
//! value).
//! 
//! The function may return with \b USBHID_RECEIVE_STARTED, indicating that a
//! receive operation is underway. The operation completes when \b size bytes
//! are received. The application should ensure that the data memory buffer be
//! available during the whole of the receive operation.
//! 
//! The function may also return with \b USBHID_RECEIVE_COMPLETED. This means that
//! the receive operation was complete by the time the function returned.
//! 
//! If the bus is not connected when the function is called, the function
//! returns \b USBHID_BUS_NOT_AVAILABLE, and no operation is begun. If \b size is 0,
//! the function returns \b USBHID_GENERAL_ERROR. If a previous receive operation
//! is already underway for this data interface, the function returns
//! \b USBHID_INTERFACE_BUSY_ERROR.
//! 
//! USB includes low-level mechanisms that ensure valid transmission of data.
//! 
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return Any of the following:
//! 		- \b USBHID_RECEIVE_STARTED: A receive operation has been
//! 			successfully started.
//! 		- \b USBHID_RECEIVE_COMPLETED: The receive operation is already 
//! 			completed.
//! 		- \b USBHID_INTERFACE_BUSY_ERROR: a previous receive operation is
//! 			underway.
//! 		- \b kUSBHID_ busNotAvailable: the bus is either suspended or
//! 			disconnected.
//! 		- \b USBHID_GENERAL_ERROR: size was zero, or other error.
//
//*****************************************************************************
uint8_t USBHID_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum);


//*****************************************************************************
//
//! Aborts an Active Recieve Operation on HID Interface.
//!
//! \param size is the number of bytes that were received and are waiting at the
//! 	assigned address.
//! \param intfNum is the data interface for which the receive should be
//! 	aborted.
//!
//! Aborts an active receive operation on HID interface \b intfNum. Returns the
//! number of bytes that were received and transferred to the data location
//! established for this receive operation. The data moved to the buffer up to
//! that time remains valid.
//! 
//! An application may choose to call this function if it decides it no longer
//! wants to receive data from the USB host. It should be noted that if a
//! continuous stream of data is being received from the host, aborting the
//! operation is akin to pressing a "pause" button; the host will be NAK'ed
//! until another receive operation is opened.
//! 
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBHID_abortReceive (uint16_t* size, uint8_t intfNum);


//*****************************************************************************
//
//! Rejects Data Received from the Host.
//!
//! This function rejects data that has been received from the host, for
//! interface \b intfNum, that does not have an active receive operation underway.
//! It resides in the USB endpoint buffer and blocks further data until a
//! receive operation is opened, or until rejected. When this function is
//! called, the buffer for this interface is purged, and the data lost. This
//! frees the USB path to resume communication.
//! 
//! See Sec. 7.2 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for a detailed discussion of
//! receive operations.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBHID_rejectData (uint8_t intfNum);


//*****************************************************************************
//
//! Aborts an Active Send Operation on Data Interface.
//!
//! \param size is the number of bytes that were sent prior to the aboert
//! 	action.
//! \param intfNum is the data interface for which the send should be
//! 	aborted.
//!
//! Aborts an active send operation on data interface \b intfNum. Returns
//! the number of bytes that were sent prior to the abort, in \b size.
//! 
//! An application may choose to call this function if sending failed, due to
//! factors such as:
//! - a surprise removal of the bus
//! - a USB suspend event
//! - any send operation that extends longer than desired//! 
//! 
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USBHID_abortSend (uint16_t* size, uint8_t intfNum);

//*****************************************************************************
//
//! Indicates the Status of the HID Interface.
//!
//! \param intfNum is the interface number for which the status is being
//! 	retrieved.
//! \param bytesSent If a send operation is underway, the number of bytes
//! 	that have been transferred to the host is returned in this location. If 
//! 	no send operation is underway, this returns zero.
//! \param bytesReceived If a receive operation is underway, the number of
//! 	bytes that have been transferred to the assigned memory location is
//! 	returned in this location. If no receive operation is underway, this
//! 	returns zero.
//!
//! Indicates the status of the HID interface \b intfNum. If a send operation is
//! active for this interface, the function also returns the number of bytes
//! that have been transmitted to the host. If a receive operation is active for
//! this interface, the function also returns the number of bytes that have been
//! received from the host and are waiting at the assigned address.
//! 
//! Because multiple flags can be returned, the possible values can be masked
//! together - for example, \b USBHID_WAITING_FOR_SEND + \b USBHID_DATA_WAITING.
//!
//! \return Any combination of the following:
//! 		- \b USBHID_WAITING_FOR_SEND: Indicates that a send operation is open
//! 			on this interface.
//! 		- \b USBHID_WAITING_FOR_RECEIVE: Indicates that a receive operation
//! 			is open on this interface.
//! 		- \b USBHID_DATA_WAITING: Indicates that data has been received from
//! 			the host for this interface, waiting in the USB receive buffers,
//! 			lacking an open receive operation to accept it.
//! 		- \b USBHID_BUS_NOT_AVAILABLE: Indicates that the bus is either
//! 			suspended or disconnected. Any operations that had previously
//! 			been underway are now aborted.
//
//*****************************************************************************
uint8_t USBHID_getInterfaceStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived);

//*****************************************************************************
//
//! Returns the Number of Bytes Waiting in the USB Endpoint Buffer.
//!
//! \param intfNum is the data interface whose buffer is to be checked.
//!
//! Returns the number of bytes waiting in the USB endpoint buffer for
//! \b intfNum. A non-zero value generally means that no receive operation is
//! open by which these bytes can be copied to a user buffer. If the value is
//! non-zero, the application should either open a receive operation so that the
//! data can be moved out of the endpoint buffer, or the data should be rejected
//! (USBHID_rejectData()).
//!
//! \return The number of bytes waiting in this buffer.
//
//*****************************************************************************
uint8_t USBHID_getBytesInUSBBuffer (uint8_t intfNum);

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

/*----------------------------------------------------------------------------
 * Event-Handling routines
 +----------------------------------------------------------------------------*/

//*****************************************************************************
//
//! \addtogroup event_handling_api
//! @{
//
//******************************************************************************

//*****************************************************************************
//
//! Data has been Received for HID Interface
//!
//! \param intfNum is which HID interface is being used.
//! 
//! This event applies to HID-Datapipe only, as opposed to HID-Traditional.
//! It indicates that data has been received for HID interface intfNum with no
//! receive operation underway. Effectively, the API doesn???t know what to do
//! with this data and is asking for instructions. The application can respond
//! by either initiating a receive operation or rejecting the data. Until one of
//! these is performed, USB data reception cannot continue; any packets received
//! from the USB host will be NAK???ed.
//! 
//! Therefore, this event should be handled quickly. A receive operation cannot
//! be started directly out of this event, since USBHID_receiveData() cannot be
//! called from the event handlers. However, the handler can set a flag for
//! main() to begin the receive operation. After this function exits, a call to
//! USBHID_getInterfaceStatus() for this HID interface will return kUSBDataWaiting.
//! 
//! If the application is written so that a receive operation is always begun
//! prior to data arriving from the host, this event will never occur. The
//! software designer generally has a choice of whether to use this event as
//! part of code flow (initiating receive operations after data is received), or
//! to always keep a receive operation open in case data arrives. (See Sec. 11
//! of \e "Programmer???s Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" more
//! discussion.)
//! 
//! \return FALSE to go asleep after interrupt (in the case the CPU slept before
//! interrupt).
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USBHID_handleDataReceived (uint8_t intfNum);

//*****************************************************************************
//
//! Send Operation on Data Interface has been Completed
//!
//! \param intfNum is which HID interface is being used.
//! 
//! This event applies to HID-Datapipe only, as opposed to HID-Traditional. It
//! indicates that a send operation on data interface intfNum has just been
//! completed.
//! 
//! In applications sending a series of large blocks of data, the designer may
//! wish to use this event to trigger another send operation. This cannot be
//! done directly out of this event, since USBHID_sendData() cannot be called
//! from the event handlers. However, the handler can set a flag for main() to
//! begin the operation.
//! 
//! \return FALSE to go asleep after interrupt (in the case the CPU slept before
//! interrupt).
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USBHID_handleSendCompleted (uint8_t intfNum);

//*****************************************************************************
//
//! Receive Operation has been Completed
//!
//! \param intfNum is which HID interface is being used.
//! 
//! This event applies to HID-Datapipe only, as opposed to HID-Traditional. It
//! indicates that a receive operation on HID interface intfNum has just been
//! completed, and the data is therefore available in the user buffer assigned
//! when the call was made to USBHID_receiveData(). If this event occurs, it
//! means that the entire buffer is full, according to the size value that was
//! requested.
//! 
//! The designer may wish to use this event to trigger another receive
//! operation. This cannot be done directly out of this event, since
//! USBHID_receiveData() cannot be called from the event handlers. However, the
//! handler can set a flag for main() to begin the operation. 
//! 
//! \return FALSE to go asleep after interrupt (in the case the CPU slept before
//! interrupt).
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USBHID_handleReceiveCompleted (uint8_t intfNum);


//*****************************************************************************
//
//! Set_Protocol Request Received from the Host
//!
//! \param protocol indicates HID_BOOT_PROTOCOL or HID_REPORT_PROTOCOL 
//! \param intfNum is which HID interface is being used.
//! 
//! This event applies to HID Traditional only. It indicates that the host has
//! requested a change in the HID protocol ??? from Boot to Standard or Standard
//! to Boot. An application that maintains separate reports for boot and
//! standard protocols can switch to the appropriate report upon receiving this
//! request. The protocol field is either HID_BOOT_PROTOCOL or
//! HID_REPORT_PROTOCOL.
//! 
//! \return FALSE to go asleep after interrupt (in the case the CPU slept before
//! interrupt).
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USBHID_handleBootProtocol (uint8_t protocol, uint8_t intfnum);

//*****************************************************************************
//
//! Set_Report request Received from the Host
//!
//! \param reportType is either USB_REQ_HID_INPUT, USB_REQ_HID_OUTPUT or
//!                             USB_REQ_HID_FEATURE
//! \param reportId is values defined by report descriptor
//! \param dataLength is length of report
//! \param intfNum is which HID interface is being used.
//! 
//! This event indicates that a Set_Report request was received from the
//! host. The application needs to supply a buffer to retrieve the report data
//! that will be sent as part of this request. This handler is passed the
//! reportType, reportId, the length of data phase as well as the interface
//! number.
//!
//! \return Pointer to input/output/feature Report Buffer
//
//*****************************************************************************
uint8_t *USBHID_handleEP0SetReport (uint8_t reportType, uint8_t reportId,
    uint16_t requestedLength,
    uint8_t intfnum);
//*****************************************************************************
//
//! Data as Part of Set_Report Request was Received from the Host
//!
//! \param intfNum is which HID interface is being used.
//! 
//! This event indicates that data as part of Set_Report request was received
//! from the host. If the application supplied a buffer as part of
//! USBHID_handleEP0SetReport, then this buffer will contain the Set Report data.
//! 
//! \return TRUE to wake up after data was received.
//
//*****************************************************************************
uint8_t USBHID_handleEP0SetReportDataAvailable (uint8_t intfnum);

//*****************************************************************************
//
//! Get_Report Request was Received from the Host
//!
//! \param reportType is either USB_REQ_HID_INPUT, USB_REQ_HID_OUTPUT or
//!                             USB_REQ_HID_FEATURE
//! \param reportId is values defined by report descriptor
//! \param requestedLength is length of report
//! \param intfNum is which HID interface is being used.
//!
//! This event indicates that a Get_Report request was received from the host.
//! The application can supply a buffer of data that will be sent to the host.
//! This handler is passed the reportType, reportId, the requested length as
//! well as the interface number.
//! 
//! \return Pointer to input input/output/feature buffer.
//
//*****************************************************************************
uint8_t *USBHID_handleEP0GetReport (uint8_t reportType, uint8_t reportId,
    uint16_t requestedLength,
    uint8_t intfnum);

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

#ifdef __cplusplus
}
#endif
#endif  //_UsbHid_H_
//Released_Version_5_20_06_03
