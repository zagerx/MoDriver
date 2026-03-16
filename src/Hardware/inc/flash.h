/**
 * @file flash.h
 * @brief STM32G473RC Flash 存储驱动头文件 - 最后两页分配 (4KB)
 * @details
 *   - 提供对 Flash 最后两页（Page 126 & 127）的读写保护
 *   - 用户区地址：0x0803F000 ~ 0x0803FFFF（共 4KB）
 *   - 支持 LUT 表等大容量数据存储
 *   - 使用双字编程确保数据完整性
 */

#ifndef _FLASH_H
#define _FLASH_H

#include <stdint.h>

/* Flash 用户区参数 - 最后两页 */
#define FLASH_USER_START_ADDR   0x0803F000UL    /* 用户区起始地址 (Page 126) */
#define FLASH_USER_END_ADDR     0x0803FFFFUL    /* 用户区结束地址 (Page 127) */
#define FLASH_USER_SIZE         4096U           /* 用户区总大小 4KB */
#define FLASH_USER_PAGE_SIZE    2048U           /* 单页大小 2KB */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 写入数据到 Flash 用户区
 * @param[in] addr 目标地址，必须在 [0x0803F000, 0x0803FFFF] 范围内
 * @param[in] data 源数据指针
 * @param[in,out] datalen 
 *   - 输入：要写入的数据长度（最大 4096 字节）
 *   - 输出：实际写入的字节数，0 表示失败
 * @note 
 *   - 写入前会自动擦除整个用户区（两页）
 *   - 使用双字（64-bit）编程
 *   - 写入后自动校验
 * @warning 写入期间会禁用中断，请确保不在中断中调用
 */
void flash_write(uint32_t addr, uint8_t *data, uint16_t *datalen);

/**
 * @brief 从 Flash 用户区读取数据
 * @param[in] addr 源地址，必须在用户区范围内
 * @param[out] data 目标缓冲区
 * @param[in,out] datalen
 *   - 输入：要读取的数据长度
 *   - 输出：实际读取的字节数
 */
void flash_read(uint32_t addr, uint8_t *data, uint16_t *datalen);

/**
 * @brief 获取用户区起始地址
 * @return Flash 用户区起始地址 0x0803F000
 */
uint32_t flash_get_user_start_address(void);

/**
 * @brief 获取用户区结束地址
 * @return Flash 用户区结束地址 0x0803FFFF
 */
uint32_t flash_get_user_end_address(void);

/**
 * @brief 获取用户区总大小
 * @return 用户区大小 4096 字节 (4KB)
 */
uint32_t flash_get_user_size(void);

/**
 * @brief 获取 Flash 单页大小
 * @return 页大小 2048 字节 (2KB)
 */
uint32_t flash_get_page_size(void);

/**
 * @brief 检查指定 Flash 区域是否为空（全 0xFF）
 * @param[in] addr 起始地址
 * @param[in] len 检查长度
 * @return 
 *   - 1: 区域为空（全 0xFF）
 *   - 0: 区域非空
 *   - -1: 参数错误（地址越界）
 */
int8_t flash_is_empty(uint32_t addr, uint16_t len);

/**
 * @brief 清空整个用户区（擦除两页为全 0xFF）
 * @return 
 *   - 0: 成功
 *   - 非0: 失败
 * @note 擦除后整区数据丢失，请谨慎使用
 */
uint8_t flash_clear_user_area(void);

/**
 * @brief 擦除用户区内的指定页
 * @param[in] page_offset 页偏移（0=第一页(Page 126)，1=第二页(Page 127)）
 * @return 
 *   - 0: 成功
 *   - 非0: 失败（参数错误或擦除失败）
 */
uint8_t flash_erase_user_page(uint8_t page_offset);

#ifdef __cplusplus
}
#endif

#endif /* _FLASH_H */
