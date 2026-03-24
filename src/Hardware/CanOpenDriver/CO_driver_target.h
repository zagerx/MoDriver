/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Hamed Jafarzadeh 	2022
 * 				Tilen Marjerle		2021
 * 				Janez Paternoster	2020
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

/* 注意：不再包含 stm32g4xx_hal.h，保持跨平台兼容性 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/**
 * \brief           CAN RX message for platform
 *
 * This is platform specific one
 */
typedef struct {
	uint32_t ident;  /*!< Standard identifier */
	uint8_t dlc;     /*!< Data length */
	uint8_t data[8]; /*!< Received data */
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t *)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t *)(msg)))->dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)(((CO_CANrxMsg_t *)(msg)))->data)

/* Received message object */
typedef struct {
	uint16_t ident;
	uint16_t mask;
	void *object;
	void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
	uint32_t ident;
	uint8_t DLC;
	uint8_t data[8];
	volatile bool_t bufferFull;
	volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
	CO_CANrx_t *rxArray;
	uint16_t rxSize;
	CO_CANtx_t *txArray;
	uint16_t txSize;
	uint16_t CANerrorStatus;
	volatile bool_t CANnormal;
	volatile bool_t useCANrxFilters;
	volatile bool_t bufferInhibitFlag;
	volatile bool_t firstCANtxMessage;
	volatile uint16_t CANtxCount;
	uint32_t errOld;

	/* Platform specific lock storage - using uintptr_t for portability */
	uintptr_t lock_send;
	uintptr_t lock_emcy;
	uintptr_t lock_od;

} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
	void *addr;
	size_t len;
	uint8_t subIndexOD;
	uint8_t attr;
	/* Additional variables (target specific) */
	void *addrNV;
} CO_storage_entry_t;

/*============================================================================
 * Critical section protection - Platform abstraction
 * 
 * These functions are implemented in the platform-specific .c file
 *===========================================================================*/

/* (un)lock critical section in CO_CANsend() */
void CO_LOCK_CAN_SEND(CO_CANmodule_t *CAN_MODULE);
void CO_UNLOCK_CAN_SEND(CO_CANmodule_t *CAN_MODULE);

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
void CO_LOCK_EMCY(CO_CANmodule_t *CAN_MODULE);
void CO_UNLOCK_EMCY(CO_CANmodule_t *CAN_MODULE);

/* (un)lock critical section when accessing Object Dictionary */
void CO_LOCK_OD(CO_CANmodule_t *CAN_MODULE);
void CO_UNLOCK_OD(CO_CANmodule_t *CAN_MODULE);

/* Synchronization between CAN receive and message processing threads. */
void CO_MemoryBarrier(void);

#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                         \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = (void *)1L;                                                                \
	} while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                       \
	do {                                                                                       \
		CO_MemoryBarrier();                                                                \
		rxNew = NULL;                                                                      \
	} while (0)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
