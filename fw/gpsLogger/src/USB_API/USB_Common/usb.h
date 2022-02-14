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
 * ======== usb.h ========
 */
#ifndef _USB_H_
#define _USB_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------+
 | Constant Definition                                                         |
 +----------------------------------------------------------------------------*/
#if defined(__TI_COMPILER_VERSION__)  || defined(__GNUC__)
#define __no_init 
#define __data16
#endif

/*----------------------------------------------------------------------------
 * The following macro names and function names are deprecated.  These were 
 * updated to new names to follow OneMCU naming convention.
 +---------------------------------------------------------------------------*/

#ifndef DEPRECATED
#define  kUSB_succeed                        USB_SUCCEED
#define  kUSB_generalError                   USB_GENERAL_ERROR
#define  kUSB_notEnabled                     USB_NOT_ENABLED
#define  kUSB_vbusPresent                    USB_VBUS_PRESENT
#define  kUSB_busActive                      USB_BUS_ACTIVE
#define  kUSB_ConnectNoVBUS                  USB_CONNECT_NO_VBUS
#define  kUSB_suspended                      USB_SUSPENDED
#define  kUSB_NotSuspended                   USB_NOT_SUSPENDED
#define  kUSB_Enumerated                     USB_ENUMERATED
#define  kUSB_purHigh                        USB_PUR_HIGH
#define  kUSB_clockFaultEvent                USB_CLOCK_FAULT_EVENT
#define  kUSB_VbusOnEvent                    USB_VBUS_ON_EVENT
#define  kUSB_VbusOffEvent                   USB_VBUS_OFF_EVENT
#define  kUSB_UsbResetEvent                  USB_RESET_EVENT
#define  kUSB_UsbSuspendEvent                USB_SUSPENDED_EVENT
#define  kUSB_UsbResumeEvent                 USB_RESUME_EVENT
#define  kUSB_dataReceivedEvent              USB_DATA_RECEIVED_EVENT
#define  kUSB_sendCompletedEvent             USB_SEND_COMPLETED_EVENT
#define  kUSB_receiveCompletedEvent          USB_RECEIVED_COMPLETED_EVENT
#define  kUSB_allUsbEvents                   USB_ALL_USB_EVENTS

#define   USB_connectionInfo                 USB_getConnectionInformation
#define   USB_connectionState  	             USB_getConnectionState
#define   USB_handleEnumCompleteEvent        USB_handleEnumerationCompleteEvent
#endif


#define FALSE   0
#define TRUE    1

#define USB_RETURN_DATA_LENGTH  8
#define SIZEOF_DEVICE_REQUEST   0x08

//Bit definitions for DEVICE_REQUEST.bmRequestType
//Bit 7:   Data direction
#define USB_REQ_TYPE_OUTPUT     0x00    //0 = Host sending data to device
#define USB_REQ_TYPE_INPUT      0x80    //1 = Device sending data to host

//Bit 6-5: Type
#define USB_REQ_TYPE_MASK       0x60    //Mask value for bits 6-5
#define USB_REQ_TYPE_STANDARD   0x00    //00 = Standard USB request
#define USB_REQ_TYPE_CLASS      0x20    //01 = Class specific
#define USB_REQ_TYPE_VENDOR     0x40    //10 = Vendor specific

//Bit 4-0: Recipient
#define USB_REQ_TYPE_RECIP_MASK 0x1F    //Mask value for bits 4-0
#define USB_REQ_TYPE_DEVICE     0x00    //00000 = Device
#define USB_REQ_TYPE_INTERFACE  0x01    //00001 = Interface
#define USB_REQ_TYPE_ENDPOINT   0x02    //00010 = Endpoint
#define USB_REQ_TYPE_OTHER      0x03    //00011 = Other

//Values for DEVICE_REQUEST.bRequest
//Standard Device Requests
#define USB_REQ_GET_STATUS              0
#define USB_REQ_CLEAR_FEATURE           1
#define USB_REQ_SET_FEATURE             3
#define USB_REQ_SET_ADDRESS             5
#define USB_REQ_GET_DESCRIPTOR          6
#define USB_REQ_SET_DESCRIPTOR          7
#define USB_REQ_GET_CONFIGURATION       8
#define USB_REQ_SET_CONFIGURATION       9
#define USB_REQ_GET_INTERFACE           10
#define USB_REQ_SET_INTERFACE           11
#define USB_REQ_SYNCH_FRAME             12

//CDC CLASS Requests
#define USB_CDC_GET_LINE_CODING         0x21
#define USB_CDC_SET_LINE_CODING         0x20
#define USB_CDC_SET_CONTROL_LINE_STATE  0x22

//HID CLASS Requests
#define USB_HID_REQ                     0x81
#define USB_REQ_GET_REPORT              0x01
#define USB_REQ_GET_IDLE                0x02
#define USB_REQ_SET_REPORT              0x09
#define USB_REQ_SET_IDLE                0x0A
#define USB_REQ_SET_PROTOCOL            0x0B
#define USB_REQ_GET_PROTOCOL            0x03

//MSC CLASS Requests
#define USB_MSC_RESET_BULK              0xFF
#define USB_MSC_GET_MAX_LUN             0xFE

// PHDC CLASS Requests
#define USB_PHDC_GET_STATUS             0x00

//HID Values for HID Report Types (tSetup.bValueH)
#define USB_REQ_HID_INPUT               0x01
#define USB_REQ_HID_OUTPUT              0x02
#define USB_REQ_HID_FEATURE             0x03

#define USB_REQ_HID_BOOT_PROTOCOL       0x00
#define USB_REQ_HID_REPORT_PROTOCOL     0x01


//Descriptor Type Values
#define DESC_TYPE_DEVICE                1       //Device Descriptor (Type 1)
#define DESC_TYPE_CONFIG                2       //Configuration Descriptor (Type 2)
#define DESC_TYPE_STRING                3       //String Descriptor (Type 3)
#define DESC_TYPE_INTERFACE             4       //Interface Descriptor (Type 4)
#define DESC_TYPE_ENDPOINT              5       //Endpoint Descriptor (Type 5)
#define DESC_TYPE_DEVICE_QUALIFIER      6       //Endpoint Descriptor (Type 6)
#define DESC_TYPE_IAD                   0x0B
#define DESC_TYPE_HUB                   0x29    //Hub Descriptor (Type 6)
#define DESC_TYPE_HID                   0x21    //HID Descriptor
#define DESC_TYPE_REPORT                0x22    //Report Descriptor
#define DESC_TYPE_PHYSICAL              0x23    //Physical Descriptor

//Feature Selector Values
#define FEATURE_REMOTE_WAKEUP           1       //Remote wakeup (Type 1)
#define FEATURE_ENDPOINT_STALL          0       //Endpoint stall (Type 0)

//Device Status Values
#define DEVICE_STATUS_REMOTE_WAKEUP     0x02
#define DEVICE_STATUS_SELF_POWER        0x01

//Maximum descriptor size
#define MAX_DESC_SIZE                   256

//DEVICE_DESCRIPTOR structure
#define SIZEOF_DEVICE_DESCRIPTOR        0x12
#define OFFSET_DEVICE_DESCRIPTOR_VID_L  0x08
#define OFFSET_DEVICE_DESCRIPTOR_VID_H  0x09
#define OFFSET_DEVICE_DESCRIPTOR_PID_L  0x0A
#define OFFSET_DEVICE_DESCRIPTOR_PID_H  0x0B
#define OFFSET_CONFIG_DESCRIPTOR_POWER  0x07
#define OFFSET_CONFIG_DESCRIPTOR_CURT   0x08

//CONFIG_DESCRIPTOR structure
#define SIZEOF_CONFIG_DESCRIPTOR 0x09

//HID DESCRIPTOR structure
//#define SIZEOF_HID_DESCRIPTOR 0x09

//Bit definitions for CONFIG_DESCRIPTOR.bmAttributes
#define CFG_DESC_ATTR_SELF_POWERED  0x40    //Bit 6: If set, device is self powered
#define CFG_DESC_ATTR_BUS_POWERED   0x80    //Bit 7: If set, device is bus powered
#define CFG_DESC_ATTR_REMOTE_WAKE   0x20    //Bit 5: If set, device supports remote wakeup

//INTERFACE_DESCRIPTOR structure
#define SIZEOF_INTERFACE_DESCRIPTOR 0x09

//ENDPOINT_DESCRIPTOR structure
#define SIZEOF_ENDPOINT_DESCRIPTOR 0x07

//Bit definitions for EndpointDescriptor.EndpointAddr
#define EP_DESC_ADDR_EP_NUM     0x0F        //Bit 3-0: Endpoint number
#define EP_DESC_ADDR_DIR_IN     0x80        //Bit 7: Direction of endpoint, 1/0 = In/Out

//Bit definitions for EndpointDescriptor.EndpointFlags
#define EP_DESC_ATTR_TYPE_MASK  0x03        //Mask value for bits 1-0
#define EP_DESC_ATTR_TYPE_CONT  0x00        //Bit 1-0: 00 = Endpoint does control transfers
#define EP_DESC_ATTR_TYPE_ISOC  0x01        //Bit 1-0: 01 = Endpoint does isochronous transfers
#define EP_DESC_ATTR_TYPE_BULK  0x02        //Bit 1-0: 10 = Endpoint does bulk transfers
#define EP_DESC_ATTR_TYPE_INT   0x03        //Bit 1-0: 11 = Endpoint does interrupt transfers

//Definition to indicate valid/invalid data
#define DATA_VALID      1
#define DATA_INVALID    0

typedef enum {
    STATUS_ACTION_NOTHING,
    STATUS_ACTION_DATA_IN,
    STATUS_ACTION_DATA_OUT
} tSTATUS_ACTION_LIST;


typedef struct _tDEVICE_REQUEST {
    uint8_t bmRequestType;         //See bit definitions below
    uint8_t bRequest;              //See value definitions below
    uint16_t wValue;                //Meaning varies with request type
    uint16_t wIndex;                //Meaning varies with request type
    uint16_t wLength;               //Number of bytes of data to transfer
} tDEVICE_REQUEST, *ptDEVICE_REQUEST;

extern __no_init tDEVICE_REQUEST __data16 tSetupPacket;
extern __no_init uint8_t __data16 abIEP0Buffer[];
extern __no_init uint8_t __data16 abOEP0Buffer[];
extern __no_init uint8_t __data16 pbXBufferAddressEp1[];
extern __no_init uint8_t __data16 pbYBufferAddressEp1[];
extern __no_init uint8_t __data16 pbXBufferAddressEp81[];
extern __no_init uint8_t __data16 pbYBufferAddressEp81[];
extern __no_init uint8_t __data16 pbXBufferAddressEp2[];
extern __no_init uint8_t __data16 pbYBufferAddressEp2[];
extern __no_init uint8_t __data16 pbXBufferAddressEp82[];
extern __no_init uint8_t __data16 pbYBufferAddressEp82[];

extern __no_init uint8_t __data16 pbXBufferAddressEp3[];
extern __no_init uint8_t __data16 pbYBufferAddressEp3[];
extern __no_init uint8_t __data16 pbXBufferAddressEp83[];
extern __no_init uint8_t __data16 pbYBufferAddressEp83[];

extern __no_init uint8_t __data16 pbXBufferAddressEp4[];
extern __no_init uint8_t __data16 pbYBufferAddressEp4[];
extern __no_init uint8_t __data16 pbXBufferAddressEp84[];
extern __no_init uint8_t __data16 pbYBufferAddressEp84[];

extern __no_init uint8_t __data16 pbXBufferAddressEp5[];
extern __no_init uint8_t __data16 pbYBufferAddressEp5[];
extern __no_init uint8_t __data16 pbXBufferAddressEp85[];
extern __no_init uint8_t __data16 pbYBufferAddressEp85[];


extern __no_init uint8_t __data16 pbXBufferAddressEp6[];
extern __no_init uint8_t __data16 pbYBufferAddressEp6[];
extern __no_init uint8_t __data16 pbXBufferAddressEp86[];
extern __no_init uint8_t __data16 pbYBufferAddressEp86[];

extern __no_init uint8_t __data16 pbXBufferAddressEp7[];
extern __no_init uint8_t __data16 pbYBufferAddressEp7[];
extern __no_init uint8_t __data16 pbXBufferAddressEp87[];
extern __no_init uint8_t __data16 pbYBufferAddressEp87[];

extern uint16_t wBytesRemainingOnIEP0;
extern uint16_t wBytesRemainingOnOEP0;
extern uint8_t abUsbRequestReturnData[];
extern uint8_t abUsbRequestIncomingData[];
extern uint8_t bEnumerationStatus;
extern uint8_t bFunctionSuspended;

//Function return values
#define USB_SUCCEED         0x00
#define USB_GENERAL_ERROR   0x01
#define USB_NOT_ENABLED     0x02
//#define kUSB_VbusNotPresent 0x03

//return values USB_getConnectionInformation(), USB_connect()
#define USB_VBUS_PRESENT     0x01
#define USB_BUS_ACTIVE       0x02    //frame sync packets are being received
#define USB_CONNECT_NO_VBUS  0x04
#define USB_SUSPENDED        0x08
#define USB_NOT_SUSPENDED    0x10
#define USB_ENUMERATED       0x20
#define USB_PUR_HIGH         0x40

//Parameters for function USB_setEnabledEvents()
#define USB_CLOCK_FAULT_EVENT         0x0001
#define USB_VBUS_ON_EVENT             0x0002
#define USB_VBUS_OFF_EVENT            0x0004
#define USB_RESET_EVENT               0x0008
#define USB_SUSPENDED_EVENT           0x0010
#define USB_RESUME_EVENT              0x0020
#define USB_DATA_RECEIVED_EVENT       0x0040
#define USB_SEND_COMPLETED_EVENT      0x0080
#define USB_RECEIVED_COMPLETED_EVENT  0x0100
#define USB_ALL_USB_EVENTS            0x01FF

//USB connection states
#define ST_USB_DISCONNECTED         0x80
#define ST_USB_CONNECTED_NO_ENUM    0x81
#define ST_ENUM_IN_PROGRESS         0x82
#define ST_ENUM_ACTIVE              0x83
#define ST_ENUM_SUSPENDED           0x84
//#define ST_FAILED_ENUM              0x85
#define ST_ERROR                    0x86
#define ST_NOENUM_SUSPENDED         0x87

#define ST_PHYS_DISCONNECTED            ST_USB_DISCONNECTED
#define ST_PHYS_CONNECTED_NOENUM        ST_USB_CONNECTED_NO_ENUM
#define ST_PHYS_CONNECTED_NOENUM_SUSP   ST_NOENUM_SUSPENDED

#define USB_CLOCKFAULT_EVENTMASK        USB_CLOCK_FAULT_EVENT
#define USB_VBUSON_EVENTMASK            USB_VBUS_ON_EVENT
#define USB_VBUSOFF_EVENTMASK           USB_VBUS_OFF_EVENT
#define USB_USBRESET_EVENTMASK          USB_RESET_EVENT
#define USB_USBSUSPEND_EVENTMASK        USB_SUSPENDED_EVENT
#define USB_USBRESUME_EVENTMASK         USB_RESUME_EVENT
#define USB_DATARECEIVED_EVENTMASK      USB_DATA_RECEIVED_EVENT
#define USB_SENDCOMPLETED_EVENTMASK     USB_SEND_COMPLETED_EVENT
#define USB_RECEIVECOMPLETED_EVENTMASK  USB_RECEIVED_COMPLETED_EVENT
#define USB_ALL_EVENTMASK           	USB_ALL_USB_EVENTS

#define SUCCESS 0
#define FAILURE 1

typedef struct _tDEVICE_REQUEST_COMPARE {
    uint8_t bmRequestType;         //See bit definitions below
    uint8_t bRequest;              //See value definitions below
    uint8_t bValueL;               //Meaning varies with request type
    uint8_t bValueH;               //Meaning varies with request type
    uint8_t bIndexL;               //Meaning varies with request type
    uint8_t bIndexH;               //Meaning varies with request type
    uint8_t bLengthL;              //Number of bytes of data to transfer (LSByte)
    uint8_t bLengthH;              //Number of bytes of data to transfer (MSByte)
    uint8_t bCompareMask;          //MSB is bRequest, if set 1, bRequest should be matched
    uint8_t (*pUsbFunction)(void); //function pointer
} tDEVICE_REQUEST_COMPARE, *ptDEVICE_REQUEST_COMPARE;

void usbStallInEndpoint(uint8_t);
void usbStallOutEndpoint(uint8_t);
void usbStallEndpoint(uint8_t);
void usbClearOEPByteCount(uint8_t);

/*----------------------------------------------------------------------------
 * These functions can be used in application
 +----------------------------------------------------------------------------*/

/*
 * MSP430 USB Module Management functions
 */

//*****************************************************************************
//
//! \addtogroup usb_api
//! @{
//
//******************************************************************************


//*****************************************************************************
//
//! Initializes the USB Module.
//!
//! 
//! Initializes the USB module by configuring power and clocks, and configures
//! pins that are critical for USB. This should be called very soon after the 
//! beginning of program execution. 
//! 
//! Note that this does not enable the USB module (that is, does not set 
//! USB_EN bit). Rather, it prepares the USB module to detect the application of
//! power to VBUS, after which the application may choose to enable the module
//! and connect to USB. Calling this function is necessary to achieve expected 
//! LPM3 current consumption into DVCC.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************

uint8_t USB_init(void);

//*****************************************************************************
//
//! Initializes the USB Module. Also enables events and connects.
//!
//! 
//! Initializes the USB module by configuring power and clocks, and configures
//! pins that are critical for USB. This should be called very soon after the 
//! beginning of program execution. 
//!
//! If connectEnable is TRUE, then this API then enables the USB module, which 
//! includes activating the PLL and setting the USB_EN bit. AFter enabling the
//! USB module, this API will connect to the host if VBUS is present.
//!
//! If eventsEnable is set to TRUE then all USB events are enabled by this API.
//!
//! \param	connectEnable	If TRUE, Connect to host if VBUS is present by 
//!							pulling the D+ signal high using the PUR pin.
//! \param  eventsEnable	If TRUE, all USB events handlers are enabled
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_setup(uint8_t connectEnable, uint8_t eventsEnable);

//*****************************************************************************
//
//! Enables the USB Module.
//!
//! Enables the USB module, which includes activating the PLL and setting the 
//! USB_EN bit. Power consumption increases as a result of this operation (see 
//! device datasheet for specifics). This call should only be made after an 
//! earlier call to USB_init(), and prior to any other call except than 
//! USB_setEnabledEvents(), or USB_getEnabledEvents(). It is usually called just
//! prior to attempting to connect with a host after a bus connection has 
//! already been detected.
//! 
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_enable ();

#ifdef USE_TIMER_FOR_RESUME

//*****************************************************************************
//
//! First phase of enabling the USB Module when USE_TIMER_FOR_RESUME is defined
//!
//! This functions is only used by USB_resume to reduce the interrupt latency
//! of the resume interrupt.
//! This function starts the XT2 crystal and then calls an event handler
//! USB_handleCrystalStartedEvent() to allow the application to get control. The
//! application can use a timer or other peripheral to "wait" for the XT2
//! crystal to stabilize. See the crystal datasheet for typical wait times.
//! The application then informs the stack of XT2
//! stabilization by calling USB_enable_PLL().
//!
//! \return \b USB_SUCCEED or USB_GENERAL_ERROR
//
//*****************************************************************************
uint8_t USB_enable_crystal (void);

//*****************************************************************************
//
//! Second phase of enabling the USB Module when USE_TIMER_FOR_RESUME is defined
//!
//! This functions is only used by USB_resume to reduce the interrupt latency
//! of the resume interrupt.
//! This function starts the PLL and then calls an event handler
//! USB_handlePLLStartedEvent() to allow the application to get control. The
//! application can use a timer or other peripheral to "wait" for the USB PLL
//! to stabilize. See the datasheet for typical PLL wait times.
//! The application then informs the stack of XT2
//! stabilization by calling USB_enable_final().
//!
//! \return \b USB_SUCCEED or USB_GENERAL_ERROR
//
//*****************************************************************************
void USB_enable_PLL(void);

//*****************************************************************************
//
//! Final phase of enabling the USB Module when USE_TIMER_FOR_RESUME is defined
//!
//! This function is only used by USB_resume to reduce the interrupt latency
//! of the resume interrupt.
//! This function gets called by the application when thye USB PLL has stabilized
//! to allow the resume process to finish.
//!
//
//*****************************************************************************
void USB_enable_final(void);

#endif

//*****************************************************************************
//
//! Disables the USB Module and PLL.
//!
//!
//! Disables the USB module and PLL. If USB is not enabled when this call is 
//! made, no error is returned - the call simply exits with success.
//! 
//! If a handleVbusOffEvent() occurs, or if USB_getConnectionState() begins 
//! returning ST_USB_DISCONNECTED, this function should be called (following a 
//! call to USB_disconnect()), in order to avoid unnecessary current draw.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_disable(void);

//*****************************************************************************
//
//! Enables/Disables the Various USB Events.
//!
//! \param events is the mask for what is to be enabled/disabled.
//!       - Valid values are:
//!        		- \b USB_CLOCK_FAULT_EVENT
//!        		- \b USB_VBUS_ON_EVENT
//!        		- \b USB_VBUS_OFF_EVENT
//!        		- \b USB_RESET_EVENT
//!        		- \b USB_SUSPENDED_EVENT
//!        		- \b USB_RESUME_EVENT
//!        		- \b USB_DATA_RECEIVED_EVENT
//!        		- \b USB_SEND_COMPLETED_EVENT
//!        		- \b USB_RECEIVED_COMPLETED_EVENT
//!        		- \b USB_ALL_USB_EVENTS
//!
//! Enables/disables various USB events. Within the events byte, all bits with
//! '1' values will be enabled, and all bits with '0' values will be disabled.
//! (There are no bit-wise operations). By default (that is, prior to any call 
//! to this function), all events are disabled.
//! 
//! The status of event enabling can be read with the USB_getEnabledEvents() 
//! function. This call can be made at any time after a call to USB_init().
//! 
//! USB_setEnabledEvents() can be thought of in a similar fashion to 
//! setting/clearing interrupt enable bits. The only benefit in keeping an event 
//! disabled is to save the unnecessary execution cycles incurred from running 
//! an "empty" event handler.
//! 
//! The mask constant \b USB_ALL_USB_EVENTS is used to enable/disable all events 
//! pertaining to core USB functions; in other words, it enables all those with 
//! a \b kUSB_ prefix. 
//! 
//! See Sec. 10 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC" for more information about
//! events.
//! 
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_setEnabledEvents (uint16_t events);


//*****************************************************************************
//
//! Returns Which Events are Enabled/Disabled.
//!
//! Returns which events are enabled and which are disabled. The definition of 
//! events is the same as for USB_enableEvents() above.
//! 
//! If the bit is set, the event is enabled. If cleared, the event is disabled. 
//! By default (that is, prior to calling USB_setEnabledEvents() ), all events 
//! are disabled. This call can be made at any time after a call to USB_init().
//! 
//! \return \b Events
//
//*****************************************************************************
uint16_t USB_getEnabledEvents ();

//*****************************************************************************
//
//! Makes USB Module Available to Host for Connection.
//!
//! Instructs the USB module to make itself available to the host for 
//! connection, by pulling the D+ signal high using the PUR pin. This call 
//! should only be made after a call to USB_enable().
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_connect ();



//*****************************************************************************
//
//! Forces a Disconnect From the USB Host.
//!
//! Forces a logical disconnect from the USB host by pulling the PUR pin low, 
//! removing the pullup on the D+ signal. The USB module and PLL remain enabled.
//! If the USB is not connected when this call is made, no error is returned -
//! the call simply exits with success after ensuring PUR is low.
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_disconnect ();
//*****************************************************************************
//
//! Resets the USB Module and the Internal State of the API.
//!
//! Resets the USB module and also the internal state of the API. The interrupt 
//! register is cleared to make sure no interrupts are pending. If the device 
//! had been enumerated, the enumeration is now lost. All open send/receive 
//! operations are aborted. 
//! 
//! This function is most often called immediately before a call to 
//! USB_connect(). It should not be called prior to USB_enable().
//!
//! \return \b USB_SUCCEED
//
//*****************************************************************************
uint8_t USB_reset ();

/**
 * Suspend USB.
 */
uint8_t USB_suspend(void);

/**
 * Resume USB.
 */
uint8_t USB_resume(void);


//*****************************************************************************
//
//! Remote Wakeup of USB Host.
//!
//! Prompts a remote wakeup of the USB host. The user must ensure that the USB 
//! descriptors had indicated remote wakeup capability (using the Descriptor 
//! Tool); otherwise the host will ignore the request.
//! 
//! If the function returns \b USB_GENERAL_ERROR, it means that the host did not 
//! grant the device the ability to perform a remote wakeup, when it enumerated 
//! the device.
//!
//! \return \b USB_SUCCEED, \b kUSBgeneralError or \b kUSB_notSuspended.
//
//*****************************************************************************
uint8_t USB_forceRemoteWakeup ();

//*****************************************************************************
//
//! Gets Connection Info.
//! 
//! Returns low-level status information about the USB connection.
//! 
//! Because multiple flags can be returned, the possible values can be masked 
//! together - for example, \b USB_VBUS_PRESENT + \b USB_SUSPENDED.
//!
//! \return A single mask that is the all the statuses together and may
//! 		consist of the following:
//! 				- \b USB_PUR_HIGH
//! 				- \b USB_SUSPENDED
//! 				- \b USB_NOT_SUSPENDED
//! 				- \b USB_ENUMERATED
//! 				- \b USB_VBUS_PRESENT
//
//*****************************************************************************
uint8_t USB_getConnectionInformation ();

//*****************************************************************************
//
//! Gets State of the USB Connection.
//!
//! Returns the state of the USB connection, according to the state diagram 
//! in Sec. 6 of \e "Programmer's Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC".
//! 
//! \return Any of the following: 
//! 			- \b ST_USB_DISCONNECTED
//! 			- \b ST_USB_CONNECTED_NO_ENUM
//! 			- \b ST_ENUM_IN_PROGRESS
//! 			- \b ST_ENUM_ACTIVE
//! 			- \b ST_ENUM_SUSPENDED
//! 			- \b ST_NOENUM_SUSPENDED,
//! 			- \b ST_ERROR.
//
//*****************************************************************************
uint8_t USB_getConnectionState ();

#ifdef NON_COMPOSITE_MULTIPLE_INTERFACES
/*
 * Switch to a different USB configuration. Used only for non-composite devices with multiple configuratons.
 */
uint8_t USB_switchInterface(uint8_t interfaceIndex);

#endif
//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

/*
 * Event-Handling routines
 */

//*****************************************************************************
//
//! \addtogroup event_handling_api
//! @{
//
//******************************************************************************


//******************************************************************************
//
//! USB PLL has Failed
//!
//! This event signals that the output of the USB PLL has failed. This event may
//! have occurred because XT2, the source of the PLL's reference clock, has
//! failed or is unreliable. If this event occurs, the USB connection will 
//! likely be lost. It is best to handle it by calling USB_disconnect() and
//! attempting a re-connection.
//! 
//! Since this event is associated with a change in state, it's a good
//! practice to return TRUE so the main loop can adapt.
//!
//! \return TRUE to keep CPU awake
//
//******************************************************************************

uint8_t USB_handleClockEvent ();

//*****************************************************************************
//
//! Valid Voltage Applied to VBUS
//!
//! If this function gets executed, it indicates that a valid voltage has been
//! applied to the VBUS pin; that is, the voltage on VBUS has transitioned from
//! low to high.
//! 
//! This usually means the device has been attached to an active USB host. It is
//! recommended to attempt a USB connection from this handler, as described in 
//! Sec. 6.3 of \e "Programmer???s Guide: MSP430 USB API Stack for CDC/PHDC/HID/MSC."
//! events.
//! 
//! Returns TRUE to wake the main loop (if LPM was entered), so that it can
//! take into account the change in state.
//!
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USB_handleVbusOnEvent ();

//*****************************************************************************
//
//! Valid Voltage Removed from VBUS
//!
//! This event indicates that a valid voltage has just been removed from the 
//! VBUS pin. That is, the voltage on VBUS has transitioned from high to low.
//! 
//! This generally means the device has been removed from an active USB host. It
//! might also mean the device is still physically attached to the host, but the
//! host went into a standby mode; or it was attached to a powered hub but the
//! host upstream from that hub became inactive. The API automatically responds
//! to a VBUS-off event by powering down the USB module and PLL, which is the
//! equivalent of calling USB_disable(). It then calls this handling function,
//! if enabled.
//! 
//! Since this event is associated with a change in state, it's a good
//! practice to return TRUE so the main loop can adapt.
//!
//! \return TRUE to keep CPU awake
//
//*****************************************************************************
uint8_t USB_handleVbusOffEvent ();

//*****************************************************************************
//
//! USB Host Issued a Reset
//!
//! This event indicates that the USB host has issued a reset of this USB
//! device. The API handles this automatically, and no action is required by the
//! application to maintain USB operation. After handling the reset, the API
//! calls this handling function, if enabled. In most cases there is no
//! significant reason for the application to respond to bus resets.
//!
//! \return TRUE
//
//*****************************************************************************
uint8_t USB_handleResetEvent ();

//*****************************************************************************
//
//! USB Host Suspends USB Device
//!
//! This event indicates that the USB host has chosen to suspend this USB device
//! after a period of active operation. It???s important that a bus-powered,
//! suspended USB device limit its consumption of power from VBUS during this
//! time. The API automatically shuts down USB-related circuitry inside the
//! MSP430???s USB module. However, the application may need to shut down other
//! circuitry drawing from VBUS. This handler is a good place to do this.
//! 
//! See Sec.11.1.3 of \e "Programmer???s Guide:
//! MSP430 USB API Stack for CDC/PHDC/HID/MSC." for a complete discussion
//! about handling suspend.
//! 
//! \returns TRUE so that the main loop can adapt.
//
//*****************************************************************************
uint8_t USB_handleSuspendEvent ();

//*****************************************************************************
//
//! USB Host has Resumed this USB Device
//!
//! This event indicates that the USB host has resumed this USB device from
//! suspend mode. If the device is bus-powered, it is no longer restricted in
//! the amount of power it can draw from VBUS. The API automatically re-enables
//! any circuitry in the MSP430???s USB module that was disabled during suspend.
//! The application can use this handler to re-enable other circuitry as well.
//! 
//! Since this event is associated with a change in state, it's a good
//! practice to return TRUE so the main loop can adapt.
//!
//! \return TRUE
//
//*****************************************************************************
uint8_t USB_handleResumeEvent ();

//*****************************************************************************
//
//! Device has Become Enumerated
//! 
//! This event indicates that the device has just become enumerated. This
//! corresponds with a state change to ST_ENUM_ACTIVE.
//! 
//! Since this event is associated with a change in state, it's a good
//! practice to return TRUE so the main loop can adapt.
//!
//! \return TRUE
//
//*****************************************************************************
uint8_t USB_handleEnumerationCompleteEvent ();

#ifdef USE_TIMER_FOR_RESUME
/*
 * When this function gets executed, it indicates that a USB_resume is in progress and the USB
 * stack requires the application to use a timer to wait until the XT2 crystal has
 * stabilized. See crystal specific datasheet for delay times. When the crystal has 
 * stabilized the application needs to call the function USB_enable_PLL() to allow
 * resume to continue.
 */
void USB_handleCrystalStartedEvent(void);

/*
 * When this function gets executed, it indicates that a USB_resume is in progress and the USB
 * stack requires the application to use a timer to wait until the USB PLL has
 * stabilized. See device specific datasheet for PLL delay times. When the PLL has 
 * stabilized the application needs to call the function USB_enable_final() to allow resume
 * to complete.
 */
void USB_handlePLLStartedEvent(void);

#endif

//******************************************************************************
//
// Close the Doxygen group.
//! @}
//
//******************************************************************************

/**
 * Send stall handshake for in- and out-endpoint0 (control pipe)
 */
void usbStallEndpoint0(void);

/**
 * Clear byte counter for endpoint0 (control pipe)
 */
void usbClearOEP0ByteCount(void);

/**
 * Send stall handshake for out-endpoint0 (control pipe)
 */
void usbStallOEP0(void);

/**
 * Send further data over control pipe if needed.
 *     Function is called from control-in IRQ. Do not call from user application
 */
void usbSendNextPacketOnIEP0(void);

/**
 * Send data over control pipe to host.
 *     Number of bytes to transmit should be set with
 *     global varible "wBytesRemainingOnIEP0" before function is called.
 */
void usbSendDataPacketOnEP0 (const uint8_t* pbBuffer);

/**
 * Receive further data from control pipe if needed.
 *     Function is called from control-out IRQ. Do not call from user application
 */
void usbReceiveNextPacketOnOEP0(void);

/**
 * Receive data from control pipe.
 *     Number of bytes to receive should be set with
 *     global varible "wBytesRemainingOnOEP0" before function is called.
 */
void usbReceiveDataPacketOnEP0 (uint8_t* pbBuffer);

/**
 * Send zero length packet on control pipe.
 */
void usbSendZeroLengthPacketOnIEP0(void);

/*Send data to host.*/
uint8_t MscSendData (const uint8_t* data, uint16_t size);

/**
 * Decode incoming usb setup packet and call corresponding function
 *     usbDecodeAndProcessUsbRequest is called from IRQ. Do not call from user application
 */
uint8_t usbDecodeAndProcessUsbRequest(void);
uint8_t usbClearEndpointFeature(void);
uint8_t usbGetConfiguration(void);
uint8_t usbGetDeviceDescriptor(void);
uint8_t usbGetConfigurationDescriptor(void);
uint8_t usbGetStringDescriptor(void);
uint8_t usbGetInterface(void);
uint8_t usbGetDeviceStatus(void);
uint8_t usbGetEndpointStatus(void);
uint8_t usbGetInterfaceStatus(void);
uint8_t usbSetAddress(void);
uint8_t usbSetConfiguration(void);
uint8_t usbClearDeviceFeature(void);
uint8_t usbSetDeviceFeature(void);
uint8_t usbSetEndpointFeature(void);
uint8_t usbSetInterface(void);
uint8_t usbInvalidRequest(void);
uint16_t usbDisableInEndpointInterrupt(uint8_t edbIndex);
void usbRestoreInEndpointInterrupt(uint16_t state);
uint16_t usbDisableOutEndpointInterrupt(uint8_t edbIndex);
void usbRestoreOutEndpointInterrupt(uint16_t state);

#define ENUMERATION_COMPLETE 0x01

/*----------------------------------------------------------------------------+
 | End of header file                                                          |
 +----------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif  /*
         * _USB_H
         *------------------------ Nothing Below This Line --------------------------
         */
//Released_Version_5_20_06_03
