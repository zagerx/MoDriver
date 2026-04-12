/**
 * @file test_flash_simple.c
 * @brief Flash 简单读写测试
 * @return 0-成功，其他-错误码
 */

#include <string.h>
#include <stdint.h>
extern uint8_t hardware_flash_clear_params_area(void);
extern void hardware_flash_write_params(uint8_t *data, uint16_t *datalen);
extern void hardware_flash_read_params(uint8_t *data, uint16_t *datalen);

/* 错误码定义 */
#define TEST_FLASH_OK         0 /* 测试通过 */
#define TEST_FLASH_ERR_WRITE  1 /* 写入失败 */
#define TEST_FLASH_ERR_READ   2 /* 读取失败 */
#define TEST_FLASH_ERR_VERIFY 3 /* 数据校验失败 */
#define TEST_FLASH_ERR_ERASE  4 /* 擦除失败 */

#define TEST_ADDR 0x0803F000UL
#define TEST_SIZE 1024

/**
 * @brief 简单 Flash 读写测试
 * @return 错误码 (0=成功)
 * @note
 *   1. 擦除后写入 TEST_SIZE 字节测试数据
 *   2. 读出并校验
 *   3. 测试完成后用户区保持擦除状态
 */
uint8_t test_flash_simple(void)
{
	uint8_t write_buf[TEST_SIZE];
	uint8_t read_buf[TEST_SIZE];
	uint16_t len;
	uint16_t i;

	/* 准备测试数据：递增序列 */
	for (i = 0; i < TEST_SIZE; i++) {
		write_buf[i] = (uint8_t)i;
	}

	/* 步骤1: 擦除用户区 */
	if (hardware_flash_clear_params_area() != 0) {
		return TEST_FLASH_ERR_ERASE;
	}

	/* 步骤2: 写入测试数据 */
	len = TEST_SIZE;
	hardware_flash_write_params(write_buf, &len);
	if (len != TEST_SIZE) {
		return TEST_FLASH_ERR_WRITE;
	}

	/* 步骤3: 读取数据 */
	memset(read_buf, 0, TEST_SIZE);
	len = TEST_SIZE;
	hardware_flash_read_params(read_buf, &len);
	if (len != TEST_SIZE) {
		return TEST_FLASH_ERR_READ;
	}

	/* 步骤4: 数据校验 */
	for (i = 0; i < TEST_SIZE; i++) {
		if (read_buf[i] != write_buf[i]) {
			return TEST_FLASH_ERR_VERIFY;
		}
	}

	/* 步骤5: 清理 - 再次擦除 */
	hardware_flash_clear_params_area();

	return TEST_FLASH_OK;
}
