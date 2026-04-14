/*
 * CANopen Object Dictionary storage object for STM32 Flash.
 *
 * @file        CO_storageBlank.c
 * @author      Janez Paternoster / Modified for Flash
 * @copyright   2021 Janez Paternoster
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

#include "CO_storageBlank.h"
#include "hardware.h"
#include <string.h>
#include "301/crc16-ccitt.h"
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
/* 签名结构 - 用于验证数据有效性 */
typedef struct {
	uint32_t magic; /* 魔数 0x5041524D ('PARM') */
	uint16_t len;   /* 数据长度 */
	uint16_t crc;   /* CRC16-CCITT */
} storage_signature_t;

#define STORAGE_MAGIC  0x5041524DUL
#define SIGNATURE_SIZE sizeof(storage_signature_t)

/* 静态缓冲区，确保 8 字节对齐 */
static uint8_t __attribute__((aligned(8))) storage_buffer[512];

/*
 * Function for writing data on "Store parameters" command - OD object 1010
 */
static ODR_t storeFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	storage_signature_t sig;
	uint16_t total_len;

	(void)CANmodule;

	/* 参数检查 */
	if (entry == NULL || entry->addr == NULL || entry->len == 0) {
		return ODR_GENERAL;
	}

	/* 检查数据是否过大 */
	if ((entry->len + SIGNATURE_SIZE) > sizeof(storage_buffer)) {
		return ODR_DATA_LONG;
	}

	/* 构造签名 */
	sig.magic = STORAGE_MAGIC;
	sig.len = (uint16_t)entry->len;
	sig.crc = crc16_ccitt(entry->addr, entry->len, 0);

	/* 组装到缓冲区（一次性写入，避免底层函数覆盖同一地址） */
	memcpy(storage_buffer, &sig, SIGNATURE_SIZE);
	memcpy(storage_buffer + SIGNATURE_SIZE, entry->addr, entry->len);

	total_len = (uint16_t)(entry->len + SIGNATURE_SIZE);

	/* 1. 擦除 Flash */
	if (hardware_flash_clear_params_area() != 0) {
		return ODR_HW;
	}

	/* 2. 一次性写入签名+数据 */
	hardware_flash_write_params(storage_buffer, &total_len);
	if (total_len != (entry->len + SIGNATURE_SIZE)) {
		return ODR_HW;
	}

	return ODR_OK;
}

/*
 * Function for restoring data on "Restore default parameters" command - OD 1011
 */
static ODR_t restoreFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
	(void)entry;
	(void)CANmodule;
	/* 擦除 Flash，标记为无效 */
	hardware_flash_clear_params_area();

	return ODR_OK;
}

CO_ReturnError_t CO_storageBlank_init(CO_storage_t *storage, CO_CANmodule_t *CANmodule,
				      OD_entry_t *OD_1010_StoreParameters,
				      OD_entry_t *OD_1011_RestoreDefaultParam,
				      CO_storage_entry_t *entries, uint8_t entriesCount,
				      uint32_t *storageInitError)
{
	CO_ReturnError_t ret;

	/* verify arguments */
	if (storage == NULL || entries == NULL || entriesCount == 0 || storageInitError == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}
	/* initialize storage and OD extensions */
	ret = CO_storage_init(storage, CANmodule, OD_1010_StoreParameters,
			      OD_1011_RestoreDefaultParam, storeFlash, restoreFlash, entries,
			      entriesCount);
	if (ret != CO_ERROR_NO) {
		return ret;
	}

	/* 启用存储 */
	storage->enabled = true;

	/* verify entries */
	*storageInitError = 0;
	for (uint8_t i = 0; i < entriesCount; i++) {
		CO_storage_entry_t *entry = &entries[i];

		/* verify arguments */
		if (entry->addr == NULL || entry->len == 0 || entry->subIndexOD < 2) {
			*storageInitError = i;
			return CO_ERROR_ILLEGAL_ARGUMENT;
		}

		/* 从 Flash 启动加载参数 */
		uint16_t read_len = (uint16_t)(entry->len + SIGNATURE_SIZE);
		if (read_len > sizeof(storage_buffer)) {
			*storageInitError |= ((uint32_t)1) << entry->subIndexOD;
			ret = CO_ERROR_DATA_CORRUPT;
			continue;
		}

		hardware_flash_read_params(storage_buffer, &read_len);
		if (read_len < (entry->len + SIGNATURE_SIZE)) {
			*storageInitError |= ((uint32_t)1) << entry->subIndexOD;
			ret = CO_ERROR_DATA_CORRUPT;
			continue;
		}

		storage_signature_t *sig = (storage_signature_t *)storage_buffer;
		bool_t data_corrupt = false;
		if (sig->magic != STORAGE_MAGIC || sig->len != entry->len) {
			data_corrupt = true;
		} else {
			uint16_t calc_crc =
				crc16_ccitt(storage_buffer + SIGNATURE_SIZE, sig->len, 0);
			if (calc_crc != sig->crc) {
				data_corrupt = true;
			} else {
				memcpy(entry->addr, storage_buffer + SIGNATURE_SIZE, entry->len);
			}
		}

		if (data_corrupt) {
			uint32_t error_bit = entry->subIndexOD;
			if (error_bit > 31U) {
				error_bit = 31U;
			}
			*storageInitError |= ((uint32_t)1) << error_bit;
			ret = CO_ERROR_DATA_CORRUPT;
		}
	}

	return ret;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
