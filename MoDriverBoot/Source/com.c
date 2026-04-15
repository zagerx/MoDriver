/************************************************************************************//**
* \file         Source/com.c
* \brief        Bootloader communication interface source file.
* \ingroup      Core
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_COM_CAN_ENABLE > 0)
#include "can.h"                                 /* can driver module                  */
#endif
#if (BOOT_COM_RS232_ENABLE > 0)
#include "rs232.h"                               /* rs232 driver module                */
#endif
#if (BOOT_COM_MBRTU_ENABLE > 0)
#include "mb.h"                                  /* modbus driver module               */
#endif
#if (BOOT_COM_USB_ENABLE > 0)
#include "usb.h"                                 /* usb driver module                  */
#endif
#if (BOOT_COM_NET_ENABLE > 0)
#include "net.h"                                 /* tcp/ip driver module               */
#endif


#if (BOOT_COM_ENABLE > 0)
/****************************************************************************************
* Macro definitions
****************************************************************************************/
#ifndef BOOT_COM_TIMEOUT_MS
/** \brief Configure the communication timeout time in milliseconds. This is the time
 *         between the sending of the last response packet and the reception of the next
 *         request packet. If this time exceeds the time configured by this macro, a
 *         timeout event is triggered. Note that this value can be overriden by another
 *         value, if added to blt_conf.h.
 */
#define BOOT_COM_TIMEOUT_MS                 (5000U)
#endif


/****************************************************************************************
* Hook functions
****************************************************************************************/
#if (BOOT_COM_CUSTOM_ENABLE > 0)
/* With this set of hook-functions, support for a new XCP communication interface can
 * be integrated. For example to support firmware updates via RS485 using a custom
 * packet format. This makes it essentially possible to use any type of communication
 * interface for firmware updates, as long as you can somehow find a way to embed an
 * XCP packet inside.
 *
 * To enable these hook-functions, add configuration macro BOOT_COM_CUSTOM_ENABLE to
 * your "blt_conf.h" and set it to a value of 1. Next, add the configuration macros
 * BOOT_COM_CUSTOM_TX_MAX_DATA and BOOT_COM_CUSTOM_RX_MAX_DATA to configure the maximum
 * size of XCP packets in bytes for transmitting and receiving, respectively.
 *
 * Afterwards implement these hook-functions for your specific custom communication
 * interface. You can reference the existing ones for an example. E.g. Rs232Xxx().
 */
extern void     ComCustomInitHook(void);
extern void     ComCustomFreeHook(void);
extern blt_bool ComCustomReceivePacketHook(blt_int8u *data, blt_int8u *len);
extern void     ComCustomTransmitPacketHook(blt_int8u *data, blt_int8u len);
#endif


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void     ComTimeoutEnable(blt_bool enable);
static blt_bool ComTimeoutDetected(void);
static void     ComTimeoutReset(void);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds the communication interface of the currently active interface. */
static tComInterfaceId comActiveInterface = COM_IF_OTHER;

/** \brief Boolean flag to enable / disable the timeout monitoring. */
static blt_bool comTimeoutMonitoringActive;

/** \brief Holds a timestamp in milliseconds when the last response packet was
 *         transmitted. Use by the communication timeout detection.
 */
static blt_int32u comLastResponseTransmitTime;


/************************************************************************************//**
** \brief     Initializes the communication module including the hardware needed for
**            the communication.
** \return    none
**
****************************************************************************************/
void ComInit(void)
{
  /* disable timeout monitoring by default. */
  ComTimeoutEnable(BLT_FALSE);  
  /* initialize the XCP communication protocol */
  XcpInit();
#if (BOOT_COM_CAN_ENABLE > 0)
  /* initialize the CAN controller */
  CanInit();
  /* set it as active */
  comActiveInterface = COM_IF_CAN;
#endif
#if (BOOT_COM_RS232_ENABLE > 0)
  /* initialize the RS232 interface */
  Rs232Init();
  /* set it as active */
  comActiveInterface = COM_IF_RS232;
#endif
#if (BOOT_COM_MBRTU_ENABLE > 0)
  /* initialize the Modbus RTU interface */
  MbRtuInit();
  /* set it as active */
  comActiveInterface = COM_IF_MBRTU;
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  /* initialize the USB interface */
  UsbInit();
  /* set it as active */
  comActiveInterface = COM_IF_USB;
#endif
#if (BOOT_COM_CUSTOM_ENABLE > 0)
  /* initialize the custom interface */
  ComCustomInitHook();
  /* set it as active */
  comActiveInterface = COM_IF_CUSTOM;
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  #if (BOOT_COM_NET_DEFERRED_INIT_ENABLE == 0)
  /* initialize the TCP/IP interface */
  NetInit();
  /* set it as active */
  comActiveInterface = COM_IF_NET;
  #endif
#endif
} /*** end of ComInit ***/


/************************************************************************************//**
** \brief     Updates the communication module by checking if new data was received
**            and submitting the request to process newly received data.
** \return    none
**
****************************************************************************************/
void ComTask(void)
{
  blt_int8u xcpPacketLen;
  /* make xcpCtoReqPacket static for runtime efficiency */
  static blt_int8u xcpCtoReqPacket[BOOT_COM_RX_MAX_DATA];
#if (BOOT_EVENTS_ENABLE > 0)
  tEventsInfoError eventsInfoError;
#endif

#if (BOOT_COM_CAN_ENABLE > 0)
  if (CanReceivePacket(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_CAN;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif
#if (BOOT_COM_RS232_ENABLE > 0)
  if (Rs232ReceivePacket(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_RS232;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif
#if (BOOT_COM_MBRTU_ENABLE > 0)
  if (MbRtuReceivePacket(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_MBRTU;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  if (UsbReceivePacket(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_USB;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif
#if (BOOT_COM_CUSTOM_ENABLE > 0)
  if (ComCustomReceivePacketHook(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_CUSTOM;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  if (NetReceivePacket(&xcpCtoReqPacket[0], &xcpPacketLen) == BLT_TRUE)
  {
    /* make this the active interface */
    comActiveInterface = COM_IF_NET;
    /* enable timeout monitoring. */
    ComTimeoutEnable(BLT_TRUE);  
    /* process packet */
    XcpPacketReceived(&xcpCtoReqPacket[0], xcpPacketLen);
  }
#endif

  /* check if a communication timeout event occurred. */
  if (ComTimeoutDetected() == BLT_TRUE)
  {
#if (BOOT_EVENTS_ENABLE > 0)
    /* trigger the OnError event.  */
    eventsInfoError.error_id = EVENT_ERROR_ID_COM_TIMEOUT;
    EventsProcess(EVENT_ID_ON_ERROR, &eventsInfoError);
#endif
  }
} /*** end of ComTask ***/


/************************************************************************************//**
** \brief     Releases the communication module.
** \return    none
**
****************************************************************************************/
void ComFree(void)
{
  /* disable timeout monitoring. */
  ComTimeoutEnable(BLT_FALSE);  
#if (BOOT_COM_USB_ENABLE > 0)
  /* disconnect the usb device from the usb host */
  UsbFree();
#endif
#if (BOOT_COM_CUSTOM_ENABLE > 0)
  /* free the custom communication interface */
  ComCustomFreeHook();
#endif
} /*** end of ComFree ***/


/************************************************************************************//**
** \brief     Transmits the packet using the xcp transport layer.
** \param     data Pointer to the byte buffer with packet data.
** \param     len  Number of data bytes that need to be transmitted.
** \return    none
**
****************************************************************************************/
void ComTransmitPacket(blt_int8u *data, blt_int16u len)
{
#if (BOOT_COM_CAN_ENABLE > 0)
  /* transmit the packet. note that len is limited to 8 in the plausibility check,
   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_CAN)
  {
    CanTransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_COM_RS232_ENABLE > 0)
  /* transmit the packet. note that len is limited to 255 in the plausibility check,
   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_RS232)
  {
    Rs232TransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_COM_MBRTU_ENABLE > 0)
  /* transmit the packet. note that len is limited to 255 in the plausibility check,
   * so cast is okay.
   */
  if (comActiveInterface == COM_IF_MBRTU)
  {
    MbRtuTransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_COM_USB_ENABLE > 0)
  /* transmit the packet */
  if (comActiveInterface == COM_IF_USB)
  {
    UsbTransmitPacket(data, len);
  }
#endif
#if (BOOT_COM_CUSTOM_ENABLE > 0)
  /* transmit the packet */
  if (comActiveInterface == COM_IF_CUSTOM)
  {
    ComCustomTransmitPacketHook(data, len);
  }
#endif
#if (BOOT_COM_NET_ENABLE > 0)
  if (comActiveInterface == COM_IF_NET)
  {
    /* transmit the packet */
    NetTransmitPacket(data, len);
  }
#endif

  /* send signal that the packet was transmitted */
  XcpPacketTransmitted();
  /* reset the communication timeout monitoring. */
  ComTimeoutReset();
} /*** end of ComTransmitPacket ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be received on the specified
**            communication interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u ComGetActiveInterfaceMaxRxLen(void)
{
  blt_int16u result;

  /* filter on communication interface identifier */
  switch (comActiveInterface)
  {
    case COM_IF_RS232:
      result = BOOT_COM_RS232_RX_MAX_DATA;
      break;

    case COM_IF_MBRTU:
      result = BOOT_COM_MBRTU_RX_MAX_DATA;
      break;

    case COM_IF_CAN:
      result = BOOT_COM_CAN_RX_MAX_DATA;
      break;

    case COM_IF_USB:
      result = BOOT_COM_USB_RX_MAX_DATA;
      break;

    case COM_IF_CUSTOM:
      result = BOOT_COM_CUSTOM_RX_MAX_DATA;
      break;

    case COM_IF_NET:
      result = BOOT_COM_NET_RX_MAX_DATA;
      break;

    default:
      result = BOOT_COM_RX_MAX_DATA;
      break;
  }

  return result;
} /*** end of ComGetActiveInterfaceMaxRxLen ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be transmitted on the
**            specified communication interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u ComGetActiveInterfaceMaxTxLen(void)
{
  blt_int16u result;

  /* filter on communication interface identifier */
  switch (comActiveInterface)
  {
    case COM_IF_RS232:
      result = BOOT_COM_RS232_TX_MAX_DATA;
      break;

    case COM_IF_MBRTU:
      result = BOOT_COM_MBRTU_TX_MAX_DATA;
      break;

    case COM_IF_CAN:
      result = BOOT_COM_CAN_TX_MAX_DATA;
      break;

    case COM_IF_USB:
      result = BOOT_COM_USB_TX_MAX_DATA;
      break;

    case COM_IF_CUSTOM:
      result = BOOT_COM_CUSTOM_TX_MAX_DATA;
      break;

    case COM_IF_NET:
      result = BOOT_COM_NET_TX_MAX_DATA;
      break;

    default:
      result = BOOT_COM_TX_MAX_DATA;
      break;
  }

  return result;
} /*** end of ComGetActiveInterfaceMaxTxLen ***/


/************************************************************************************//**
** \brief     This function obtains the XCP connection state.
** \return    BLT_TRUE when an XCP connection is established, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool ComIsConnected(void)
{
  blt_bool result = BLT_FALSE;

  /* Is there an active XCP connection? This indicates that the communication interface
   * is in the connection state. 
   */  
  if (XcpIsConnected())
  {
    result = BLT_TRUE;
  }
#if (ADDON_GATEWAY_MOD_ENABLE > 0)
  /* Is the gateway active? This indicates an XCP connection with a slave. */
  if (GatewayIsActive())
  {
    result = BLT_TRUE;
  }
#endif
  /* give the result back to the caller. */
  return result;
} /*** end of ComIsConnected ***/


/************************************************************************************//**
** \brief     Enables or disables the communication timeout monitoring.
** \param     enable BLT_TRUE to enabe, BLT_FALSE to disable.
** \return    none
**
****************************************************************************************/
static void ComTimeoutEnable(blt_bool enable)
{
  /* update the flag and reset. */
  comTimeoutMonitoringActive = enable;
  ComTimeoutReset();
} /*** end of ComTimeoutEnable ***/


/************************************************************************************//**
** \brief     Determines if a communication timeout was detected.
** \return    BLT_TRUE if a communication timeout was detected, BLT_FALSE otherwise.
**
****************************************************************************************/
static blt_bool ComTimeoutDetected(void)
{
  blt_bool   result = BLT_FALSE;
  blt_int32u currentTime;
  blt_int32u deltaTime;

  /* only need to monitor for communication timeouts if something is connected and the
   * timeout monitoring is enabled.
   */
  if ( (ComIsConnected() == BLT_TRUE) && (comTimeoutMonitoringActive == BLT_TRUE) )
  {
    /* determine the delta time between the last packet transmission and now. note that
     * this also works in case of a timer overflow, due to integer math.
     */
    currentTime = TimerGet();
    deltaTime = currentTime - comLastResponseTransmitTime;
    /* did a communication timeout occur? */
    if (deltaTime >= BOOT_COM_TIMEOUT_MS)
    {
      /* disable the timeout monitoring. otherwise the timeout event keeps triggering.
       * note that the timeout monitoring is enabled again upon reception of a new
       * response packet.
      */
      ComTimeoutEnable(BLT_FALSE);
      /* timeout detected. Update the result. */
      result = BLT_TRUE;
    }
  }

  /* give the result back to the caller. */
  return result;
} /*** end of ComTimeoutDetected ***/


/************************************************************************************//**
** \brief     Resets the communication timeout time. Should be called each time a 
**            response packet is transmitted.
** \return    none
**
****************************************************************************************/
static void ComTimeoutReset(void)
{
  /* update the timestamp when the last response packet was transmitted. */
  comLastResponseTransmitTime = TimerGet();
} /*** end of ComTimeoutReload ***/


#if (BOOT_COM_DEFERRED_INIT_ENABLE == 1)
/************************************************************************************//**
** \brief     The deferred init feature makes it possible to bypass the initialization of
**            a communication interface until this function is called. This feature can
**            be enabled for a specific communication interface via macro
**            BOOT_COM_XXX_DEFERRED_INIT_ENABLE in blt_conf.h. At this point only the NET
**            communication interface supports this feature, as its initialization can
**            take quite a long time. If there is a valid user program present, then this
**            would cause an unwanted delay after each reset before the user program can
**            be started.
** \attention Note that when this feature is enabled for a communication interface, the
**            communication interface is only enabled when: (a) no valid user program is
**            present or (b) when CpuUserProgramStartHook() returns BLT_FALSE. This means
**            that after a normal reactivation of the bootloader from the user program,
**            the communication interface is not initialized and firmware updates are
**            not possible! In this case it is recommended to somehow pass on the
**            communication initialization request from the user program to the
**            bootloader. When this request detected by the bootloader application, this
**            function should be called. EEPROM or shared RAM can be used to pass on such
**            a request.
** \return    none
**
****************************************************************************************/
void ComDeferredInit(void)
{
#if (BOOT_COM_NET_ENABLE > 0)
  #if (BOOT_COM_NET_DEFERRED_INIT_ENABLE == 1)
  /* perform deferred initialization the TCP/IP interface */
  NetDeferredInit();
  /* set it as active */
  comActiveInterface = COM_IF_NET;
  #endif
#endif
#if (BOOT_BACKDOOR_HOOKS_ENABLE == 0)
  /* the default internal timed backdoor mechanism should start its timer after the
   * communication interfaces are initialized. since a deferred initialization was now
   * performed, the backdoor timer should be restarted.
   */
  BackDoorRestartTimer();
#endif
} /*** end of ComDeferredInit ***/
#endif /* BOOT_COM_DEFERRED_INIT_ENABLE == 1 */


#endif /* BOOT_COM_ENABLE > 0 */

/*********************************** end of com.c **************************************/
