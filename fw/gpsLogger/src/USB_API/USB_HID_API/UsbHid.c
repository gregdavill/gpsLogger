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
/** @file UsbHid.c
 *  @brief Contains APIs related to HID (Human Interface Device) device class.
 */
/* 
 * ======== UsbHid.c ========
 */

#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "UsbHid.h"
#include <descriptors.h>
#include <string.h>
#ifdef _HID_

//function pointers
extern void *(*USB_TX_memcpy)(void * dest, const void * source, size_t count);
extern void *(*USB_RX_memcpy)(void * dest, const void * source, size_t count);

//Local Macros
#define INTFNUM_OFFSET(X)   (X - HID0_INTFNUM)  //Get the HID offset

extern uint8_t const report_len_input[HID_NUM_INTERFACES];

static struct _HidWrite {
    uint16_t nHidBytesToSend;                       //holds counter of bytes to be sent
    uint16_t nHidBytesToSendLeft;                   //holds counter how many bytes is still to be sent
    const uint8_t* pHidBufferToSend;               //holds the buffer with data to be sent
    uint8_t bCurrentBufferXY;                      //indicates which buffer is to use next for for write into IN OUT endpoint
} HidWriteCtrl[HID_NUM_INTERFACES];

static struct _HidRead {
    uint8_t *pUserBuffer;                          //holds the current position of user's receiving buffer. If NULL- no receiving
                                                //operation started
    uint8_t *pCurrentEpPos;                        //current positon to read of received data from curent EP
    uint16_t nBytesToReceive;                       //holds how many bytes was requested by receiveData() to receive
    uint16_t nBytesToReceiveLeft;                   //holds how many bytes is still requested by receiveData() to receive
    uint8_t * pCT1;                                //holds current EPBCTxx register
    uint8_t * pCT2;                                //holds next EPBCTxx register
    uint8_t * pEP2;                                //holds addr of the next EP buffer
    uint8_t nBytesInEp;                            //how many received bytes still available in current EP
    uint8_t bCurrentBufferXY;                      //indicates which buffer is used by host to transmit data via OUT endpoint
} HidReadCtrl[HID_NUM_INTERFACES];

extern uint16_t wUsbEventMask;

uint8_t hidProtocol[HID_NUM_INTERFACES] = {0};
uint8_t hidIdleRate[HID_NUM_INTERFACES] = {0};

/*----------------------------------------------------------------------------+
 | Global Variables                                                            |
 +----------------------------------------------------------------------------*/

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern __no_init tEDB __data16 tOutputEndPointDescriptorBlock[];


void HidCopyUsbToBuff (uint8_t* pEP, uint8_t* pCT, uint8_t);

/*----------------------------------------------------------------------------+
 | Functions' implementatin                                                    |
 +----------------------------------------------------------------------------*/

//resets internal HID data structure
void HidResetData ()
{
    int16_t i;

    //indicates which buffer is used by host to transmit data via OUT endpoint3 - X buffer is first
    //HidReadCtrl[intfIndex].bCurrentBufferXY = X_BUFFER;

    memset(&HidReadCtrl, 0, sizeof(HidReadCtrl));
    memset(&HidWriteCtrl, 0, sizeof(HidWriteCtrl));
    for (i = 0; i < HID_NUM_INTERFACES; i++){
        hidProtocol[i] = USB_REQ_HID_REPORT_PROTOCOL;
    }
}

/*
 * Sends a pre-built report reportData to the host.
 * Returns:  USBHID_SEND_COMPLETE
 *          USBHID_INTERFACE_BUSY_ERROR
 *          kUSBHID_busSuspended
 */
uint8_t USBHID_sendReport (const uint8_t * reportData, uint8_t intfNum)
{
    uint8_t byte_count;
    uint8_t * pEP1;
    uint8_t * pCT1;

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        return (USBHID_BUS_NOT_AVAILABLE);
    }

    if (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    }

    byte_count = report_len_input[INTFNUM_OFFSET(intfNum)];

    if (*pCT1 & EPBCNT_NAK){                                                        //if this EP is empty
        USB_TX_memcpy(pEP1, reportData, byte_count);                                //copy data into IEP X or Y buffer
        *pCT1 = byte_count;                                                         //Set counter for usb In-Transaction
        HidWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;    //switch buffer
        return (USBHID_SEND_COMPLETE);
    }
    return (USBHID_INTERFACE_BUSY_ERROR);
}

/*
 * Receives report reportData from the host.
 * Return:     USBHID_RECEIVE_COMPLETED
 *          USBHID_GENERAL_ERROR
 *          kUSBHID_busSuspended
 */
uint8_t USBHID_receiveReport (uint8_t * reportData, uint8_t intfNum)
{
    uint8_t ret = USBHID_GENERAL_ERROR;
    uint8_t nTmp1 = 0;

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        return (USBHID_BUS_NOT_AVAILABLE);
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer

            USB_RX_memcpy(reportData, HidReadCtrl[INTFNUM_OFFSET(
                                                      intfNum)].pCurrentEpPos,
                nTmp1);
            //memcpy(reportData, HidReadCtrl.pEP1, nTmp1);
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
            *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 = 0;                 //clear NAK, EP ready to receive data

            ret = USBHID_RECEIVE_COMPLETED;
        }
    }
    return (ret);
}

/*
 * Sends data over interface intfNum, of size size and starting at address data.
 * Returns:  USBHID_SEND_STARTED
 *          USBHID_SEND_COMPLETE
 *          USBHID_INTERFACE_BUSY_ERROR
 */
uint8_t USBHID_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum)
{
	uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (USBHID_GENERAL_ERROR);
    }

    state = usbDisableInEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreInEndpointInterrupt(state);
        return (USBHID_BUS_NOT_AVAILABLE);
    }

    if (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
    	usbRestoreInEndpointInterrupt(state);
        return (USBHID_INTERFACE_BUSY_ERROR);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSend = size;
    HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft = size;
    HidWriteCtrl[INTFNUM_OFFSET(intfNum)].pHidBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    usbRestoreInEndpointInterrupt(state);

    return (USBHID_SEND_STARTED);
}


//this function is used only by USB interrupt
int16_t HidToHostFromBuffer (uint8_t intfNum)
{
    uint8_t byte_count, nTmp2;
    uint8_t * pEP1;
    uint8_t * pEP2;
    uint8_t * pCT1;
    uint8_t * pCT2;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft == 0){    //do we have somtething to send?
        HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSend = 0;

        //call event callback function
        if (wUsbEventMask & USB_SEND_COMPLETED_EVENT){
            bWakeUp = USBHID_handleSendCompleted(intfNum);
        }
        return (bWakeUp);
    }

    if (!(tInputEndPointDescriptorBlock[edbIndex].bEPCNF & EPCNF_TOGGLE)){
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].iep_Y_Buffer;
        pCT1 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        pEP2 = (uint8_t*)stUsbHandle[intfNum].iep_X_Buffer;
        pCT2 = &tInputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can send over one endpoint buffer
    //2 bytes a reserved: [0] - HID Report Descriptor, [1] - count of valid bytes
    byte_count =
        (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft >
         EP_MAX_PACKET_SIZE -
         2) ? EP_MAX_PACKET_SIZE -
        2 : HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1 + 2, HidWriteCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pHidBufferToSend,
            byte_count);                                                        //copy data into IEP3 X or Y buffer
        pEP1[0] = 0x3F;                                                         //set HID report descriptor: 0x3F
        pEP1[1] = byte_count;                                                   //set HID report descriptor

        //64 bytes will be send: we use only one HID report descriptor
        *pCT1 = 0x40;                                                           //Set counter for usb In-Transaction

        HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft -= byte_count;
        HidWriteCtrl[INTFNUM_OFFSET(intfNum)].pHidBufferToSend += byte_count;   //move buffer pointer

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft > 0) &&  //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){                                              //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft >
                 EP_MAX_PACKET_SIZE -
                 2) ? EP_MAX_PACKET_SIZE -
                2 : HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft;

            USB_TX_memcpy(pEP2 + 2, HidWriteCtrl[INTFNUM_OFFSET(
                                                     intfNum)].pHidBufferToSend,
                byte_count);                                                    //copy data into IEP3 X or Y buffer
            pEP2[0] = 0x3F;                                                     //set HID report descriptor: 0x3F
            pEP2[1] = byte_count;                                               //set byte count of valid data

            //64 bytes will be send: we use only one HID report descriptor
            *pCT2 = 0x40;                                                       //Set counter for usb In-Transaction

            HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft -=
                byte_count;
            HidWriteCtrl[INTFNUM_OFFSET(intfNum)].pHidBufferToSend +=
                byte_count;                                                     //move buffer pointer
        }
    }
    return (bWakeUp);
}



/*
 * Aborts an active send operation on interface intfNum.  Returns the number of bytes that were sent prior to the abort, in size.
 */
uint8_t USBHID_abortSend (uint16_t* size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableInEndpointInterrupt(edbIndex);

    *size =
        (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSend -
         HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft);
    HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSend = 0;
    HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft = 0;

    usbRestoreInEndpointInterrupt(state);
    return (USB_SUCCEED);
}

//This function copies data from OUT endpoint into user's buffer
//Arguments:
//pEP - pointer to EP to copy from
//pCT - pointer to pCT control reg
//
void HidCopyUsbToBuff (uint8_t* pEP, uint8_t* pCT,uint8_t intfNum)
{
    uint8_t nCount;

    //how many byte we can get from one endpoint buffer
    nCount =
        (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
         HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp) ? HidReadCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesInEp : HidReadCtrl[INTFNUM_OFFSET(
                                                                  intfNum)].
        nBytesToReceiveLeft;

    USB_RX_memcpy(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer, pEP, nCount);   //copy data from OEPx X or Y buffer
    HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft -= nCount;
    HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer += nCount;                     //move buffer pointer
    //to read rest of data next time from this place

    if (nCount == HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                 //all bytes are copied from receive buffer?
        //switch current buffer
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;

        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;

        //clear NAK, EP ready to receive data
        *pCT = 0;
    } else {
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp -= nCount;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos = pEP + nCount;
    }
}


/*
 * Receives data over interface intfNum, of size size, into memory starting at address data.
 */

uint8_t USBHID_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t nTmp1;
    uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                      //read size is 0
        (data == NULL)){
        return (USBHID_GENERAL_ERROR);
    }

    state = usbDisableOutEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
    	usbRestoreOutEndpointInterrupt(state);
        return (USBHID_BUS_NOT_AVAILABLE);
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){          //receive process already started
    	usbRestoreOutEndpointInterrupt(state);
        return (USBHID_RECEIVE_IN_PROGRESS);
    }

    HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;            //bytes to receive
    HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;        //left bytes to receive
    HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                //set user receive buffer

    //read rest of data from buffer, if any
    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){
        //copy data from pEP-endpoint into User's buffer
        HidCopyUsbToBuff(HidReadCtrl[INTFNUM_OFFSET(
                                         intfNum)].pCurrentEpPos,
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);

        if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            USBHID_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
            usbRestoreOutEndpointInterrupt(state);
            return (USBHID_RECEIVE_COMPLETED);                              //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else {
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        }
        nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                            //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                *(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos + 1);  //holds how many valid bytes in the EP buffer
            if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1 - 2){
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1 - 2;
            }
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos += 2;        //here starts user data
            HidCopyUsbToBuff(HidReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pCurrentEpPos,
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);
        }

        if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBHID_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBHID_RECEIVE_COMPLETED);                              //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                *(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos + 1);  //holds how many valid bytes in the EP buffer
            if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1 - 2){
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1 - 2;
            }
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos += 2;        //here starts user data
            HidCopyUsbToBuff(HidReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pCurrentEpPos,
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1,intfNum);

            nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                      //do we have more data to receive?
                (nTmp1 & EPBCNT_NAK)){                                      //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                       //clear NAK bit
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp =
                    *(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 + 1);       //holds how many valid bytes in the EP buffer
                if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1 -
                    2){
                    HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1 - 2;
                }
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 += 2;             //here starts user data
                HidCopyUsbToBuff(HidReadCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pEP2,
                    HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2,intfNum);
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //the Receive opereation is completed
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            USBHID_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
        }
        usbRestoreOutEndpointInterrupt(state);
        return (USBHID_RECEIVE_COMPLETED);
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (USBHID_RECEIVE_STARTED);
}


//this function is used only by USB interrupt.
//It fills user receiving buffer with received data
int16_t HidToBufferFromHost (uint8_t intfNum)
{
    uint8_t * pEP1;
    uint8_t nTmp1;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //do we have somtething to receive?
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        return (bWakeUp);
    }

    //No data to receive...
    if (!((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX |
           tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY)
          & 0x80)){
        return (bWakeUp);
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //X is current buffer
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can get from one endpoint buffer
    nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

    if (nTmp1 & EPBCNT_NAK){
        nTmp1 = nTmp1 & 0x7f;                                                   //clear NAK bit
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = *(pEP1 + 1);          //holds how many valid bytes in the EP buffer
        if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1 - 2){
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1 - 2;
        }
        pEP1 += 2;                                                              //here starts user data
        HidCopyUsbToBuff(pEP1, HidReadCtrl[INTFNUM_OFFSET(
                                               intfNum)].pCT1,intfNum);

        nTmp1 = *HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        //try read data from second buffer
        if ((HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft > 0) &&   //do we have more data to send?
            (nTmp1 & EPBCNT_NAK)){                                              //if the second buffer has received data as indicated by setting of
                                                                                //EPBCNT_NAK = 1.  If EPBCNT_NAK is zero - no data
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 
               *(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 + 1);  //holds how many valid bytes in the EP buffer.  BUG#15751
            if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > nTmp1 - 2){
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1 - 2;
            }
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 += 2;                     //here starts user data
            HidCopyUsbToBuff(HidReadCtrl[INTFNUM_OFFSET(
                                             intfNum)].pEP2,
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2,intfNum);
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        }
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){         //the Receive opereation is completed
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;                //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            bWakeUp = USBHID_handleReceiveCompleted(intfNum);
        }

        if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                   //Is not read data still available in the EP?
            if (wUsbEventMask & USB_DATA_RECEIVED_EVENT){
                bWakeUp = USBHID_handleDataReceived(intfNum);
            }
        }
    }
    return (bWakeUp);
}

//helper for USB interrupt handler
int16_t HidIsReceiveInProgress (uint8_t intfNum)
{
    return (HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL);
}

/*
 * Aborts an active receive operation on interface intfNum.
 * size: the number of bytes that were received and transferred
 * to the data location established for this receive operation.
 */

uint8_t USBHID_abortReceive (uint16_t* size, uint8_t intfNum)
{
    uint16_t state;
    uint8_t edbIndex;
	
    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    *size = 0;                                                                  //set received bytes count to 0

    //is receive operation underway?
    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        //how many bytes are already received?
        *size = HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;

        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = 0;
    }

    //restore interrupt status
    usbRestoreOutEndpointInterrupt(state);
    return (USB_SUCCEED);
}

/*
 * This function rejects payload data that has been received from the host.
 */

uint8_t USBHID_rejectData (uint8_t intfNum)
{
	uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableOutEndpointInterrupt(edbIndex);

    //interrupts disable

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if (bFunctionSuspended){
    	usbRestoreOutEndpointInterrupt(state);
        return (USBHID_BUS_NOT_AVAILABLE);
    }

    //Is receive operation underway?
    //- do not flush buffers if any operation still active.
    if (!HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        uint8_t tmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                    EPBCNT_NAK;
        uint8_t tmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                    EPBCNT_NAK;

        if (tmp1 ^ tmp2){                                                       //switch current buffer if any and only ONE of the
                                                                                //buffers is full
            //switch current buffer
            HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (HidReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
        }

        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;                   //flush buffer X
        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;                   //flush buffer Y
        HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;                    //indicates that no more data available in the EP
    }

    usbRestoreOutEndpointInterrupt(state);
    return (USB_SUCCEED);
}

/*
 * This function indicates the status of the interface intfNum.
 * If a send operation is active for this interface,
 * the function also returns the number of bytes that have been transmitted to the host.
 * If a receiver operation is active for this interface, the function also returns
 * the number of bytes that have been received from the host and are waiting at the assigned address.
 *
 * returns USBHID_WAITING_FOR_SEND (indicates that a call to USBHID_SendData()
 * has been made, for which data transfer has not been completed)
 *
 * returns USBHID_WAITING_FOR_RECEIVE (indicates that a receive operation
 * has been initiated, but not all data has yet been received)
 *
 * returns USBHID_DATA_WAITING (indicates that data has been received
 * from the host, waiting in the USB receive buffers)
 */

uint8_t USBHID_getInterfaceStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived)
{
    uint8_t ret = 0;
    uint16_t stateIn, stateOut;
    uint8_t edbIndex;

    *bytesSent = 0;
    *bytesReceived = 0;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    stateIn = usbDisableInEndpointInterrupt(edbIndex);
    stateOut = usbDisableOutEndpointInterrupt(edbIndex);

    //Is send operation underway?
    if (HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft != 0){
        ret |= USBHID_WAITING_FOR_SEND;
        *bytesSent = HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSend -
                     HidWriteCtrl[INTFNUM_OFFSET(intfNum)].nHidBytesToSendLeft;
    }

    //Is receive operation underway?
    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){
        ret |= USBHID_WAITING_FOR_RECEIVE;
        *bytesReceived = HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                         HidReadCtrl[INTFNUM_OFFSET(intfNum)].
                         nBytesToReceiveLeft;
    } else {                                                                    //not receive operation started
        //do not access USB memory if suspended (PLL off).
        //It may produce BUS_ERROR
        if (!bFunctionSuspended){
            if ((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                 EPBCNT_NAK)  |                                                 //any of buffers has a valid data packet
                (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                 EPBCNT_NAK)){
                ret |= USBHID_DATA_WAITING;
            }
        }
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated  - report no other tasks pending
        ret = USBHID_BUS_NOT_AVAILABLE;
    }

    //restore interrupt status
    usbRestoreInEndpointInterrupt(stateIn);
    usbRestoreOutEndpointInterrupt(stateOut);

    return (ret);
}

/*
 * Returns how many bytes are in the buffer are received and ready to be read.
 */
uint8_t USBHID_getBytesInUSBBuffer (uint8_t intfNum)
{
    uint8_t bTmp1 = 0;
    uint8_t bTmp2;

    uint8_t edbIndex;
    uint16_t state;


    edbIndex = stUsbHandle[intfNum].edb_Index;

    //interrupts disable
    state = usbDisableOutEndpointInterrupt(edbIndex);

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report 0 bytes available
    	usbRestoreOutEndpointInterrupt(state);
        return (0);
    }

    if (HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){                   //If a RX operation is underway, part of data may
                                                                                //was read of the OEP buffer
        bTmp1 = HidReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp;
        if (*HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & EPBCNT_NAK){           //the next buffer has a valid data packet
            bTmp2 = *(HidReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 + 1);           //holds how many valid bytes in the EP buffer
            if (bTmp2 >
                (*HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F) - 2){       //check if all data received correctly
                bTmp1 +=
                    (*HidReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F) - 2;
            } else {
                bTmp1 += bTmp2;
            }
        }
    } else {
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){     //this buffer has a valid data packet
            bTmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & 0x7F;
            bTmp1 = *((uint8_t*)stUsbHandle[intfNum].oep_X_Buffer + 1);
            if (bTmp2 - 2 < bTmp1){                                             //check if the count (second byte) is valid
                bTmp1 = bTmp2 - 2;
            }
        }
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){     //this buffer has a valid data packet
            bTmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & 0x7F;
            if (bTmp2 - 2 > *((uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer + 1)){   //check if the count (second byte) is valid
                bTmp1 += *((uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer + 1);
            } else {
                bTmp1 += bTmp2 - 2;
            }
        }
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (bTmp1);
}


#endif //ifdef _HID_

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
//Released_Version_5_20_06_03
