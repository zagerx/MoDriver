/************************************************************************************//**
* \file         Demo/ARMCM4_STM32G4_Nucleo_G474RE_GCC/Boot/hooks.c
* \brief        Bootloader callback source file.
* \ingroup      Boot_ARMCM4_STM32G4_Nucleo_G474RE_GCC
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2021  by Feaser    http://www.feaser.com    All rights reserved
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
#include "led.h"                                 /* LED driver header                  */
#include "stm32g4xx.h"                           /* STM32 CPU and HAL header           */
#include "stm32g4xx_ll_gpio.h"                   /* STM32 LL GPIO header               */

/** \brief Magic key for bootloader activation via RAM. */
#define BOOTLOADER_MAGIC_KEY  0xDEADBEEF
/** \brief RAM address to store the magic key. */
volatile uint32_t *bootFlag = (volatile uint32_t *)0x2000FFF0;


/****************************************************************************************
*   C P U   D R I V E R   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_CPU_USER_PROGRAM_START_HOOK > 0)
/************************************************************************************//**
** \brief     Callback that gets called when the bootloader is about to exit and
**            hand over control to the user program. This is the last moment that
**            some final checking can be performed and if necessary prevent the
**            bootloader from activiting the user program.
** \return    BLT_TRUE if it is okay to start the user program, BLT_FALSE to keep
**            keep the bootloader active.
**
****************************************************************************************/
blt_bool CpuUserProgramStartHook(void)
{
  /* clean up the LED driver */
  LedBlinkExit();
  /*  okay to start the user program.*/
  return BLT_TRUE;
} /*** end of CpuUserProgramStartHook ***/
#endif /* BOOT_CPU_USER_PROGRAM_START_HOOK > 0 */


/****************************************************************************************
*   W A T C H D O G   D R I V E R   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_COP_HOOKS_ENABLE > 0)
/************************************************************************************//**
** \brief     Callback that gets called at the end of the internal COP driver
**            initialization routine. It can be used to configure and enable the
**            watchdog.
** \return    none.
**
****************************************************************************************/
void CopInitHook(void)
{
  /* this function is called upon initialization. might as well use it to initialize
   * the LED driver. It is kind of a visual watchdog anyways.
   */
  LedBlinkInit(100);
} /*** end of CopInitHook ***/


/************************************************************************************//**
** \brief     Callback that gets called at the end of the internal COP driver
**            service routine. This gets called upon initialization and during
**            potential long lasting loops and routine. It can be used to service
**            the watchdog to prevent a watchdog reset.
** \return    none.
**
****************************************************************************************/
void CopServiceHook(void)
{
  /* run the LED blink task. this is a better place to do it than in the main() program
   * loop. certain operations such as flash erase can take a long time, which would cause
   * a blink interval to be skipped. this function is also called during such operations,
   * so no blink intervals will be skipped when calling the LED blink task here.
   */
  LedBlinkTask();
} /*** end of CopServiceHook ***/
#endif /* BOOT_COP_HOOKS_ENABLE > 0 */


/****************************************************************************************
*   B A C K D O O R   E N T R Y   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_BACKDOOR_HOOKS_ENABLE > 0)
/************************************************************************************//**
** \brief     Initializes the backdoor entry option.
** \return    none.
**
****************************************************************************************/
void BackDoorInitHook(void)
{
} /*** end of BackDoorInitHook ***/


/************************************************************************************//**
** \brief     Checks if a backdoor entry is requested.
** \return    BLT_TRUE if the backdoor entry is requested, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool BackDoorEntryHook(void)
{
  /* check if the application requested a firmware update via RAM magic key */
  if (*bootFlag == BOOTLOADER_MAGIC_KEY)
  {
    *bootFlag = 0;  /* clear flag to prevent staying here on next power-up */
    return BLT_TRUE; /* stay in bootloader for firmware update */
  }
  /* no firmware update request, so allow the bootloader to start the user program */
  return BLT_FALSE;
} /*** end of BackDoorEntryHook ***/
#endif /* BOOT_BACKDOOR_HOOKS_ENABLE > 0 */


/****************************************************************************************
*   N O N - V O L A T I L E   M E M O R Y   D R I V E R   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_NVM_HOOKS_ENABLE > 0)
/************************************************************************************//**
** \brief     Callback that gets called at the start of the internal NVM driver
**            initialization routine.
** \return    none.
**
****************************************************************************************/
void NvmInitHook(void)
{
} /*** end of NvmInitHook ***/


/************************************************************************************//**
** \brief     Callback that gets called at the start of the NVM driver write
**            routine. It allows additional memory to be operated on. If the address
**            is not within the range of the additional memory, then
**            BLT_NVM_NOT_IN_RANGE must be returned to indicate that the data hasn't
**            been written yet.
** \param     addr Start address.
** \param     len  Length in bytes.
** \param     data Pointer to the data buffer.
** \return    BLT_NVM_OKAY if successful, BLT_NVM_NOT_IN_RANGE if the address is
**            not within the supported memory range, or BLT_NVM_ERROR is the write
**            operation failed.
**
****************************************************************************************/
blt_int8u NvmWriteHook(blt_addr addr, blt_int32u len, blt_int8u *data)
{
  return BLT_NVM_NOT_IN_RANGE;
} /*** end of NvmWriteHook ***/


/************************************************************************************//**
** \brief     Callback that gets called at the start of the NVM driver erase
**            routine. It allows additional memory to be operated on. If the address
**            is not within the range of the additional memory, then
**            BLT_NVM_NOT_IN_RANGE must be returned to indicate that the memory
**            hasn't been erased yet.
** \param     addr Start address.
** \param     len  Length in bytes.
** \return    BLT_NVM_OKAY if successful, BLT_NVM_NOT_IN_RANGE if the address is
**            not within the supported memory range, or BLT_NVM_ERROR is the erase
**            operation failed.
**
****************************************************************************************/
blt_int8u NvmEraseHook(blt_addr addr, blt_int32u len)
{
  return BLT_NVM_NOT_IN_RANGE;
} /*** end of NvmEraseHook ***/


/************************************************************************************//**
** \brief     Callback that gets called at the end of the NVM programming session.
** \return    BLT_TRUE is successful, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool NvmDoneHook(void)
{
  return BLT_TRUE;
} /*** end of NvmDoneHook ***/
#endif /* BOOT_NVM_HOOKS_ENABLE > 0 */


#if (BOOT_NVM_CHECKSUM_HOOKS_ENABLE > 0)
/************************************************************************************//**
** \brief     Verifies the checksum, which indicates that a valid user program is
**            present and can be started.
** \return    BLT_TRUE if successful, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool NvmVerifyChecksumHook(void)
{
  return BLT_TRUE;
} /*** end of NvmVerifyChecksum ***/


/************************************************************************************//**
** \brief     Writes a checksum of the user program to non-volatile memory. This is
**            performed once the entire user program has been programmed. Through
**            the checksum, the bootloader can check if a valid user programming is
**            present and can be started.
** \return    BLT_TRUE if successful, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool NvmWriteChecksumHook(void)
{
  return BLT_TRUE;
}
#endif /* BOOT_NVM_CHECKSUM_HOOKS_ENABLE > 0 */


/****************************************************************************************
*   E V E N T S   M O D U L E   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_EVENTS_ENABLE > 0)
/***********************************************************************************//**
** \brief     Callback that gets called when a firmware update related event gets
**            triggered. Implement your event handling here. For example for updating
**            a user interface or for logging purposes.
**            Cast the opaque info pointer to the correct structure type (tEventsInfoXxx)
**            to access additional event related information.
** \param     id The identifier of the event that occurred.
** \param     info Opaque pointer to event identifier related information. Can be 
**            BLT_NULL depending on the event identifer. For example when no additional
**            information available for the event.
** \return    none
**
****************************************************************************************/
void EventsHook(tEventsId id, void const *info)
{
  blt_addr         base_addr;
  blt_int32u       num_bytes;
  blt_int8u        progress;
  blt_char const * filename;
  tEventsErrorId   error_id;
  static blt_bool  update_from_file = BLT_FALSE;

  /* Filter on the events identifier. */
  switch (id)
  {
    /* Event EVENT_ID_ON_ENTRY triggers once after a power-on or reset event, when the
     * bootloader finished it's initialization.
     */
    case EVENT_ID_ON_ENTRY:
      break;

    /* Event EVENT_ID_ON_START triggers at the start of a firmware update. Info
     * parameter:
     *  filename: The filename for the firmware updates. Only applicable for firmware
     *            updates from a locally attached FAT filesystem (e.g. SD-card). For
     *            firmware updates via a communication interface (e.g. RS232, CAN, etc.)
     *            the parameter value is BLT_NULL.
     */
    case EVENT_ID_ON_START:
      update_from_file = BLT_FALSE;
      filename = ((tEventsInfoStart const *)info)->filename;
      if (filename != BLT_NULL)
      {
        update_from_file = BLT_TRUE;
      }
      break;

    /* Event EVENT_ID_ON_ERASE triggers each time when a part of non-volatile memory is
     * about to be erase. Info parameters:
     *   base_addr: The start memory address of the erase operation.
     *   num_bytes: The number of bytes that are to be erased, starting at base_addr.
     */
    case EVENT_ID_ON_ERASE:
      base_addr = ((tEventsInfoErase const *)info)->base_addr;
      num_bytes = ((tEventsInfoErase const *)info)->num_bytes;
      break;

    /* Event EVENT_ID_ON_WRITE triggers each time when a part of non-volatile memory is
     * about to be programmed. Info parameters:
     *   base_addr: The start memory address of the program operation.
     *   num_bytes: The number of bytes that are to be programmed, starting at base_addr.
     *   progress:  Overall firmware update progress as a percentage (0..100).
     */
    case EVENT_ID_ON_WRITE:
      base_addr = ((tEventsInfoWrite const *)info)->base_addr;
      num_bytes = ((tEventsInfoWrite const *)info)->num_bytes;
      progress  = ((tEventsInfoWrite const *)info)->progress;
      break;

    /* Event EVENT_ID_ON_SUCCESS triggers after the firmware update successfully
     * completed.
     */
    case EVENT_ID_ON_SUCCESS:
      break;

    /* Event EVENT_ID_ON_ERROR triggers upon detection of an error during the firmware
     * update. Info parameter:
     *  error_id: The error identifier. Refer to tEventsErrorId for a list of available
     *            error identifiers (EVENT_ERROR_ID_xxx) and their meaning.
     */
    case EVENT_ID_ON_ERROR:
      error_id = ((tEventsInfoError const *)info)->error_id;
      break;


    /* Event EVENT_ID_ON_SUPPRESS triggers when the bootloader intended to start the
     * user program, yet decided against it. This can for example happen when the
     * checksum verification failed or the logic in CpuUserProgramStartHook() requested
     * the bootloader to stay active.
     */
    case EVENT_ID_ON_SUPPRESS:
      break;

    /* Event EVENT_ID_ON_EXIT triggers when the bootloader is about to hand over control
     * to the user program by starting it.
     */
    case EVENT_ID_ON_EXIT:
      break;

    default:
      break;  
  }
} /*** end of EventsHook ***/
#endif /* BOOT_EVENTS_ENABLE > 0 */


/****************************************************************************************
*   S E E D / K E Y   S E C U R I T Y   H O O K   F U N C T I O N S
****************************************************************************************/

#if (BOOT_XCP_SEED_KEY_ENABLE > 0)
/************************************************************************************//**
** \brief     Provides a seed to the XCP master that will be used for the key
**            generation when the master attempts to unlock the specified resource.
**            Called by the GET_SEED command.
** \param     resource  Resource that the seed if requested for (XCP_RES_XXX).
** \param     seed      Pointer to byte buffer wher the seed will be stored.
** \return    Length of the seed in bytes.
**
****************************************************************************************/
blt_int8u XcpGetSeedHook(blt_int8u resource, blt_int8u *seed)
{
  /* request seed for unlocking ProGraMming resource */
  if ((resource & XCP_RES_PGM) != 0)
  {
    seed[0] = 0x55;
  }

  /* return seed length */
  return 1;
} /*** end of XcpGetSeedHook ***/


/************************************************************************************//**
** \brief     Called by the UNLOCK command and checks if the key to unlock the
**            specified resource was correct. If so, then the resource protection
**            will be removed.
** \param     resource  resource to unlock (XCP_RES_XXX).
** \param     key       pointer to the byte buffer holding the key.
** \param     len       length of the key in bytes.
** \return    1 if the key was correct, 0 otherwise.
**
****************************************************************************************/
blt_int8u XcpVerifyKeyHook(blt_int8u resource, blt_int8u *key, blt_int8u len)
{
  /* suppress compiler warning for unused parameter */
  len = len;

  /* the example key algorithm in "libseednkey.dll" works as follows:
   *  - PGM will be unlocked if key = seed - 1
   */

  /* check key for unlocking ProGraMming resource */
  if ((resource == XCP_RES_PGM) && (key[0] == (0x55-1)))
  {
    /* correct key received for unlocking PGM resource */
    return 1;
  }

  /* still here so key incorrect */
  return 0;
} /*** end of XcpVerifyKeyHook ***/
#endif /* BOOT_XCP_SEED_KEY_ENABLE > 0 */


/*********************************** end of hooks.c ************************************/
