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
 * ======== UsbCdc.c ========
 */
#include <descriptors.h>

#ifdef _CDC_


#include "../USB_Common/device.h"
#include "../USB_Common/defMSP430USB.h"
#include "../USB_Common/usb.h"                  //USB-specific Data Structures
#include "../USB_CDC_API/UsbCdc.h"

#include <string.h>

//Local Macros
#define INTFNUM_OFFSET(X)   (X - CDC0_INTFNUM)  //Get the CDC offset

static struct _CdcControl {
    uint32_t lBaudrate;
    uint8_t bDataBits;
    uint8_t bStopBits;
    uint8_t bParity;
} CdcControl[CDC_NUM_INTERFACES];

extern uint16_t wUsbEventMask;

//function pointers
extern void *(*USB_TX_memcpy)(void * dest, const void * source, size_t count);
extern void *(*USB_RX_memcpy)(void * dest, const void * source, size_t count);


/*----------------------------------------------------------------------------+
 | Global Variables                                                            |
 +----------------------------------------------------------------------------*/

extern __no_init tEDB __data16 tInputEndPointDescriptorBlock[];
extern __no_init tEDB __data16 tOutputEndPointDescriptorBlock[];


void CdcResetData ()
{
    int16_t i;

    //indicates which buffer is used by host to transmit data via OUT endpoint3 - X buffer is first
    //CdcReadCtrl[intfIndex].bCurrentBufferXY = X_BUFFER;

    memset(&CdcWriteCtrl, 0, sizeof(CdcWriteCtrl));
    memset(&CdcReadCtrl, 0, sizeof(CdcReadCtrl));
    memset(&CdcControl, 0, sizeof(CdcControl));

    for (i = 0; i < CDC_NUM_INTERFACES; i++){
        CdcControl[i].bDataBits = 8;
    }
}
/*
 * Sends data over interface intfNum, of size size and starting at address data.

 * Returns:  USBCDC_SEND_STARTED
 *          USBCDC_SEND_COMPLETE
 *          USBCDC_INTERFACE_BUSY_ERROR
 */


uint8_t USBCDC_sendData (const uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (size == 0){
        return (USBCDC_GENERAL_ERROR);
    }

    state = usbDisableInEndpointInterrupt(edbIndex);

    //do not access USB memory if suspended (PLL uce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreInEndpointInterrupt(state);                                            //restore interrupt status
        return (USBCDC_BUS_NOT_AVAILABLE);
    }

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft != 0){
        //the USB still sends previous data, we have to wait
    	usbRestoreInEndpointInterrupt(state);                                           //restore interrupt status
        return (USBCDC_INTERFACE_BUSY_ERROR);
    }

    //This function generate the USB interrupt. The data will be sent out from interrupt

    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = size;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft = size;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend = data;

    //trigger Endpoint Interrupt - to start send operation
    USBIEPIFG |= 1 << (edbIndex + 1);                                       //IEPIFGx;

    usbRestoreInEndpointInterrupt(state);

    return (USBCDC_SEND_STARTED);
}


#define EP_MAX_PACKET_SIZE_CDC      0x40

//this function is used only by USB interrupt
int16_t CdcToHostFromBuffer (uint8_t intfNum)
{
    uint8_t byte_count, nTmp2;
    uint8_t * pEP1;
    uint8_t * pEP2;
    uint8_t * pCT1;
    uint8_t * pCT2;
    uint8_t bWakeUp = FALSE;                                                   //TRUE for wake up after interrupt
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft == 0){    //do we have somtething to send?
        if (!CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent){        //zero packet was not yet sent
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = TRUE;

            if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend ==
                EP_MAX_PACKET_SIZE_CDC){
                if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY ==
                    X_BUFFER){
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;
                        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = Y_BUFFER;                                     //switch buffer
                    }
                } else {
                    if (tInputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                        EPBCNT_NAK){
                        tInputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;
                        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY
                            = X_BUFFER;                                     //switch buffer
                    }
                }
            }

            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = 0;      //nothing to send

            //call event callback function
            if (wUsbEventMask & USB_SEND_COMPLETED_EVENT){
                bWakeUp = USBCDC_handleSendCompleted(intfNum);
            }
        } //if (!bSentZeroPacket)

        return (bWakeUp);
    }

    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bZeroPacketSent = FALSE;          //zero packet will be not sent: we have data

    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){
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
    byte_count =
        (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft >
         EP_MAX_PACKET_SIZE_CDC) ? EP_MAX_PACKET_SIZE_CDC : CdcWriteCtrl[
            INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;
    nTmp2 = *pCT1;

    if (nTmp2 & EPBCNT_NAK){
        USB_TX_memcpy(pEP1, CdcWriteCtrl[INTFNUM_OFFSET(
                                             intfNum)].pUsbBufferToSend,
            byte_count);                                                            //copy data into IEP3 X or Y buffer
        *pCT1 = byte_count;                                                         //Set counter for usb In-Transaction
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;    //switch buffer
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft -= byte_count;
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend += byte_count;       //move buffer pointer
        CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;

        //try to send data over second buffer
        nTmp2 = *pCT2;
        if ((CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft > 0) &&      //do we have more data to send?
            (nTmp2 & EPBCNT_NAK)){                                                  //if the second buffer is free?
            //how many byte we can send over one endpoint buffer
            byte_count =
                (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft >
                 EP_MAX_PACKET_SIZE_CDC) ? EP_MAX_PACKET_SIZE_CDC :
                CdcWriteCtrl[
                    INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;

            USB_TX_memcpy(pEP2, CdcWriteCtrl[INTFNUM_OFFSET(
                                                 intfNum)].pUsbBufferToSend,
                byte_count);                                                        //copy data into IEP3 X or Y buffer
            *pCT2 = byte_count;                                                     //Set counter for usb In-Transaction
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;                                                         //switch buffer
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft -=
                byte_count;
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].pUsbBufferToSend +=
                byte_count;                                                         //move buffer pointer
            CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].last_ByteSend = byte_count;
        }
    }
    return (bWakeUp);
}

/*
 * Aborts an active send operation on interface intfNum.  Returns the number of bytes that were sent prior to the abort, in size.
 */


uint8_t USBCDC_abortSend (uint16_t* size, uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableInEndpointInterrupt(edbIndex);                                                         //disable interrupts - atomic operation

    *size =
        (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend -
         CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft);
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend = 0;
    CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft = 0;

    usbRestoreInEndpointInterrupt(state);
    return (USB_SUCCEED);
}


//This function copies data from OUT endpoint into user's buffer
//Arguments:
//pEP - pointer to EP to copy from
//pCT - pointer to pCT control reg
//
void CopyUsbToBuff (uint8_t* pEP, uint8_t* pCT, uint8_t intfNum)
{
    uint8_t nCount;

    //how many byte we can get from one endpoint buffer
    nCount =
        (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
         CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp) ? CdcReadCtrl[
            INTFNUM_OFFSET(intfNum)].nBytesInEp : CdcReadCtrl[INTFNUM_OFFSET(
                                                                  intfNum)].
        nBytesToReceiveLeft;

    USB_RX_memcpy(CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer, pEP, nCount);   //copy data from OEP3 X or Y buffer
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft -= nCount;
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer += nCount;                     //move buffer pointer
    //to read rest of data next time from this place

    if (nCount == CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                 //all bytes are copied from receive buffer?
        //switch current buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
            (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY + 1) & 0x01;

        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;

        //clear NAK, EP ready to receive data
        *pCT = 0x00;
    } else {
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp -= nCount;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos = pEP + nCount;
    }
}
/*
 * Receives data over interface intfNum, of size size, into memory starting at address data.
 */


uint8_t USBCDC_receiveData (uint8_t* data, uint16_t size, uint8_t intfNum)
{
    uint8_t nTmp1;
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if ((size == 0) ||                                                      //read size is 0
        (data == NULL)){
        return (USBCDC_GENERAL_ERROR);
    }

    state = usbDisableOutEndpointInterrupt(edbIndex);
    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //data can not be read because of USB suspended
    	usbRestoreOutEndpointInterrupt(state);
        return (USBCDC_BUS_NOT_AVAILABLE);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){          //receive process already started
    	usbRestoreOutEndpointInterrupt(state);
        return (USBCDC_INTERFACE_BUSY_ERROR);
    }

    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive = size;            //bytes to receive
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = size;        //left bytes to receive
    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = data;                //set user receive buffer

    //read rest of data from buffer, if any
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){
        //copy data from pEP-endpoint into User's buffer
        CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                      intfNum)].pCurrentEpPos,
            CdcReadCtrl[INTFNUM_OFFSET(
                            intfNum)
            ].pCT1, intfNum);

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBCDC_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBCDC_RECEIVE_COMPLETED);                              //receive completed
        }

        //check other EP buffer for data - exchange pCT1 with pCT2
        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 ==
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX){
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        } else {
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        }

        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        //try read data from second buffer
        if (nTmp1 & EPBCNT_NAK){                                            //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer
            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);
        }

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){ //the Receive opereation is completed
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;        //no more receiving pending
            if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
                USBCDC_handleReceiveCompleted(intfNum);                     //call event handler in interrupt context
            }
            usbRestoreOutEndpointInterrupt(state);
            return (USBCDC_RECEIVE_COMPLETED);                              //receive completed
        }
    } //read rest of data from buffer, if any

    //read 'fresh' data, if available
    nTmp1 = 0;
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //this is current buffer
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            //this is the active EP buffer
            //pEP1
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

            //second EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    } else {                                                                //Y_BUFFER
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){
            //this is the active EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCurrentEpPos =
                (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

            //second EP buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
                (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
                &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
            nTmp1 = 1;                                                      //indicate that data is available
        }
    }

    if (nTmp1){
        //how many byte we can get from one endpoint buffer
        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        while (nTmp1 == 0)
        {
            nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;
        }

        if (nTmp1 & EPBCNT_NAK){
            nTmp1 = nTmp1 & 0x7f;                                           //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;        //holds how many valid bytes in the EP buffer

            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pCurrentEpPos,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);

            nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            //try read data from second buffer
            if ((CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft >
                 0) &&                                                      //do we have more data to send?
                (nTmp1 & EPBCNT_NAK)){                                      //if the second buffer has received data?
                nTmp1 = nTmp1 & 0x7f;                                       //clear NAK bit
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;    //holds how many valid bytes in the EP buffer
                CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                              intfNum)].pEP2,
                    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                    CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
            }
        }
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //the Receive opereation is completed
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            USBCDC_handleReceiveCompleted(intfNum);                         //call event handler in interrupt context
        }
        usbRestoreOutEndpointInterrupt(state);
        return (USBCDC_RECEIVE_COMPLETED);
    }

    //interrupts enable
    usbRestoreOutEndpointInterrupt(state);
    return (USBCDC_RECEIVE_STARTED);
}

//this function is used only by USB interrupt.
//It fills user receiving buffer with received data
int16_t CdcToBufferFromHost (uint8_t intfNum)
{
    uint8_t * pEP1;
    uint8_t nTmp1;
    uint8_t bWakeUp = FALSE;                                                   //per default we do not wake up after interrupt

    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){     //do we have somtething to receive?
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;            //no more receiving pending
        return (bWakeUp);
    }

    //No data to receive...
    if (!((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX |
           tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY)
          & 0x80)){
        return (bWakeUp);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY == X_BUFFER){ //X is current buffer
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;

        //second EP buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;
    } else {
        //this is the active EP buffer
        pEP1 = (uint8_t*)stUsbHandle[intfNum].oep_Y_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY;

        //second EP buffer
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pEP2 =
            (uint8_t*)stUsbHandle[intfNum].oep_X_Buffer;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 =
            &tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX;
    }

    //how many byte we can get from one endpoint buffer
    nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1;

    if (nTmp1 & EPBCNT_NAK){
        nTmp1 = nTmp1 & 0x7f;                                                   //clear NAK bit
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;                //holds how many valid bytes in the EP buffer

        CopyUsbToBuff(pEP1, CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1, intfNum);

        nTmp1 = *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        //try read data from second buffer
        if ((CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft > 0) &&   //do we have more data to send?
            (nTmp1 & EPBCNT_NAK)){                                              //if the second buffer has received data?
            nTmp1 = nTmp1 & 0x7f;                                               //clear NAK bit
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = nTmp1;            //holds how many valid bytes in the EP buffer
            CopyUsbToBuff(CdcReadCtrl[INTFNUM_OFFSET(
                                          intfNum)].pEP2,
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2, intfNum);
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT1 =
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2;
        }
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft == 0){         //the Receive opereation is completed
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;                //no more receiving pending
        if (wUsbEventMask & USB_RECEIVED_COMPLETED_EVENT){
            bWakeUp |= USBCDC_handleReceiveCompleted(intfNum);
        }

        if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp){                   //Is not read data still available in the EP?
            if (wUsbEventMask & USB_DATA_RECEIVED_EVENT){
                bWakeUp |= USBCDC_handleDataReceived(intfNum);
            }
        }
    }
    return (bWakeUp);
}

//helper for USB interrupt handler
int16_t CdcIsReceiveInProgress (uint8_t intfNum)
{
    return (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL);
}
/*
 * Aborts an active receive operation on interface intfNum.
 * size: the number of bytes that were received and transferred
 * to the data location established for this receive operation.
 */


uint8_t USBCDC_abortReceive (uint16_t* size, uint8_t intfNum)
{
    //interrupts disable
	uint8_t edbIndex;
	uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    *size = 0;                                                              //set received bytes count to 0

    //is receive operation underway?
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        //how many bytes are already received?
        *size = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft;

        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer = NULL;
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceiveLeft = 0;
    }

    //restore interrupt status
    usbRestoreOutEndpointInterrupt(state);
    return (USB_SUCCEED);
}
/*
 * This function rejects payload data that has been received from the host.
 */

uint8_t USBCDC_rejectData (uint8_t intfNum)
{
    uint8_t edbIndex;
    uint16_t state;

    edbIndex = stUsbHandle[intfNum].edb_Index;
    state = usbDisableOutEndpointInterrupt(edbIndex);

    //atomic operation - disable interrupts

    //do not access USB memory if suspended (PLL off). It may produce BUS_ERROR
    if (bFunctionSuspended){
    	usbRestoreOutEndpointInterrupt(state);
        return (USBCDC_BUS_NOT_AVAILABLE);
    }

    //Is receive operation underway?
    //- do not flush buffers if any operation still active.
    if (!CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer){
        uint8_t tmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                    EPBCNT_NAK;
        uint8_t tmp2 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                    EPBCNT_NAK;

        if (tmp1 ^ tmp2){                                                   //switch current buffer if any and only ONE of buffers
                                                                            //is full
            //switch current buffer
            CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY =
                (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].bCurrentBufferXY +
                 1) & 0x01;
        }

        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX = 0;               //flush buffer X
        tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY = 0;               //flush buffer Y
        CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp = 0;                //indicates that no more data available in the EP
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
 * returns USBCDC_WAITING_FOR_SEND (indicates that a call to USBCDC_SendData()
 * has been made, for which data transfer has not been completed)
 *
 * returns USBCDC_WAITING_FOR_RECEIVE (indicates that a receive operation
 * has been initiated, but not all data has yet been received)
 *
 * returns USBCDC_DATA_WAITING (indicates that data has been received
 * from the host, waiting in the USB receive buffers)
 */


uint8_t USBCDC_getInterfaceStatus (uint8_t intfNum, uint16_t* bytesSent, uint16_t* bytesReceived)
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
    if (CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft != 0){
        ret |= USBCDC_WAITING_FOR_SEND;
        *bytesSent = CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSend -
                     CdcWriteCtrl[INTFNUM_OFFSET(intfNum)].nCdcBytesToSendLeft;
    }

    //Is receive operation underway?
    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pUserBuffer != NULL){
        ret |= USBCDC_WAITING_FOR_RECEIVE;
        *bytesReceived = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesToReceive -
                         CdcReadCtrl[INTFNUM_OFFSET(intfNum)].
                         nBytesToReceiveLeft;
    } else {                                                                //receive operation not started
        //do not access USB memory if suspended (PLL off).
        //It may produce BUS_ERROR
        if (!bFunctionSuspended){
            if ((tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX &
                 EPBCNT_NAK)  |                                             //any of buffers has a valid data packet
                (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY &
                 EPBCNT_NAK)){
                ret |= USBCDC_DATA_WAITING;
            }
        }
    }

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
        //if suspended or not enumerated - report no other tasks pending
        ret = USBCDC_BUS_NOT_AVAILABLE;
    }

    //restore interrupt status
    usbRestoreInEndpointInterrupt(stateIn);
    usbRestoreOutEndpointInterrupt(stateOut);

    __no_operation();
    return (ret);
}

/*
 * Returns how many bytes are in the buffer are received and ready to be read.
 */
uint8_t USBCDC_getBytesInUSBBuffer (uint8_t intfNum)
{
    uint8_t bTmp1 = 0;
    uint16_t state;
    uint8_t edbIndex;

    edbIndex = stUsbHandle[intfNum].edb_Index;

    state = usbDisableOutEndpointInterrupt(edbIndex);
    //atomic operation - disable interrupts

    if ((bFunctionSuspended) ||
        (bEnumerationStatus != ENUMERATION_COMPLETE)){
    	usbRestoreOutEndpointInterrupt(state);
        //if suspended or not enumerated - report 0 bytes available
        return (0);
    }

    if (CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp > 0){               //If a RX operation is underway, part of data may
                                                                            //was read of the OEP buffer
        bTmp1 = CdcReadCtrl[INTFNUM_OFFSET(intfNum)].nBytesInEp;
        if (*CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & EPBCNT_NAK){       //the next buffer has a valid data packet
            bTmp1 += *CdcReadCtrl[INTFNUM_OFFSET(intfNum)].pCT2 & 0x7F;
        }
    } else {
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 = tOutputEndPointDescriptorBlock[edbIndex].bEPBCTX & 0x7F;
        }
        if (tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & EPBCNT_NAK){ //this buffer has a valid data packet
            bTmp1 += tOutputEndPointDescriptorBlock[edbIndex].bEPBCTY & 0x7F;
        }
    }

    usbRestoreOutEndpointInterrupt(state);
    return (bTmp1);
}


//----------------------------------------------------------------------------
//Line Coding Structure
//dwDTERate     | 4 | Data terminal rate, in bits per second
//bCharFormat   | 1 | Stop bits, 0 = 1 Stop bit, 1 = 1,5 Stop bits, 2 = 2 Stop bits
//bParityType   | 1 | Parity, 0 = None, 1 = Odd, 2 = Even, 3= Mark, 4 = Space
//bDataBits     | 1 | Data bits (5,6,7,8,16)
//----------------------------------------------------------------------------
uint8_t usbGetLineCoding (void)
{
    uint8_t infIndex;

    if(tSetupPacket.wIndex % 2)
    {
        infIndex = (tSetupPacket.wIndex-1) / 2;
    }
    else
    {
        infIndex = (tSetupPacket.wIndex) / 2;
    }

    abUsbRequestReturnData[6] =
        CdcControl[infIndex].bDataBits;          //Data bits = 8
    abUsbRequestReturnData[5] =
        CdcControl[infIndex].bParity;            //No Parity
    abUsbRequestReturnData[4] =
        CdcControl[infIndex].bStopBits;          //Stop bits = 1

    abUsbRequestReturnData[3] =
        CdcControl[infIndex].lBaudrate >> 24;
    abUsbRequestReturnData[2] =
        CdcControl[infIndex].lBaudrate >> 16;
    abUsbRequestReturnData[1] =
        CdcControl[infIndex].lBaudrate >> 8;
    abUsbRequestReturnData[0] =
        CdcControl[infIndex].lBaudrate;

    wBytesRemainingOnIEP0 = 0x07;                                           //amount of data to be send over EP0 to host
    usbSendDataPacketOnEP0((uint8_t*)&abUsbRequestReturnData[0]);              //send data to host

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t usbSetLineCoding (void)
{
    usbReceiveDataPacketOnEP0((uint8_t*)&abUsbRequestIncomingData);            //receive data over EP0 from Host

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t usbSetControlLineState (void)
{
	USBCDC_handleSetControlLineState((uint8_t)tSetupPacket.wIndex,
            (uint8_t)tSetupPacket.wValue);
    usbSendZeroLengthPacketOnIEP0();                                        //Send ZLP for status stage

    return (FALSE);
}

//----------------------------------------------------------------------------

uint8_t Handler_SetLineCoding (void)
{
    uint8_t bWakeUp;
    volatile uint8_t infIndex;

    if(tSetupPacket.wIndex % 2)
    {
        infIndex = (tSetupPacket.wIndex-1) / 2;
    }
    else
    {
        infIndex = (tSetupPacket.wIndex) / 2;
    }

    //Baudrate Settings

    CdcControl[infIndex].lBaudrate =
        (uint32_t)abUsbRequestIncomingData[3] << 24 |
        (uint32_t)abUsbRequestIncomingData[2] << 16 |
        (uint32_t)
        abUsbRequestIncomingData[1] << 8 | abUsbRequestIncomingData[0];
    bWakeUp =
        USBCDC_handleSetLineCoding(tSetupPacket.wIndex,
            CdcControl[infIndex].lBaudrate);

    return (bWakeUp);
}

#endif  //ifdef _CDC_


/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
//Released_Version_5_20_06_03
