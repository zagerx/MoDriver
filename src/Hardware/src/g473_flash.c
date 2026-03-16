/**
 * @file g473_flash.c
 * @brief STM32G473RC Flash 存储驱动 - 最后两页分配 (4KB)
 * @details
 *   - 芯片型号: STM32G473RC
 *   - Flash 容量: 256KB (128 页 × 2KB)
 *   - 使用最后两页: Page 126 + Page 127
 *   - 地址范围: 0x0803F000 ~ 0x0803FFFF (共 4KB)
 *   - 页大小: 2KB
 */

#include "flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* Flash 物理参数 */
#define FLASH_BASE_ADDR         0x08000000UL
#define FLASH_PAGE_SIZE         2048U           /* 2KB per page */
#define FLASH_TOTAL_SIZE        (256U * 1024U)  /* 256KB total */
#define FLASH_PAGE_NUM_TOTAL    (FLASH_TOTAL_SIZE / FLASH_PAGE_SIZE) /* 128 pages */

/* 用户数据区配置 - 使用最后两页 */
#define FLASH_USER_PAGE_COUNT   2               /* 使用 2 页 */
#define FLASH_USER_START_PAGE   (FLASH_PAGE_NUM_TOTAL - FLASH_USER_PAGE_COUNT) /* Page 126 */
#define FLASH_USER_START_ADDR   (FLASH_BASE_ADDR + (FLASH_USER_START_PAGE * FLASH_PAGE_SIZE))
#define FLASH_USER_SIZE         (FLASH_USER_PAGE_COUNT * FLASH_PAGE_SIZE) /* 4096 bytes */
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + FLASH_USER_SIZE - 1)

/* Flash 操作超时 */
#define FLASH_TIMEOUT_MS        5000U

/* 错误码定义 */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR_ADDR,         /* 地址非法 */
    FLASH_ERROR_SIZE,         /* 数据大小非法 */
    FLASH_ERROR_UNLOCK,       /* 解锁失败 */
    FLASH_ERROR_ERASE,        /* 擦除失败 */
    FLASH_ERROR_PROGRAM,      /* 编程失败 */
    FLASH_ERROR_LOCK,         /* 锁定失败 */
    FLASH_ERROR_VERIFY,       /* 校验失败 */
    FLASH_ERROR_NULL_PTR,     /* 空指针 */
} flash_status_t;

/* 内部函数声明 */
static flash_status_t flash_unlock(void);
static flash_status_t flash_lock(void);
static flash_status_t flash_erase_page(uint32_t page_addr);
static flash_status_t flash_program_doubleword(uint32_t addr, uint64_t data);
static uint32_t flash_get_page(uint32_t addr);
static flash_status_t flash_verify(uint32_t addr, uint8_t *data, uint16_t len);

/**
 * @brief 检查地址是否合法（必须在用户区内）
 * @param addr 要检查的地址
 * @return FLASH_OK 或 FLASH_ERROR_ADDR
 */
static flash_status_t flash_check_address(uint32_t addr)
{
    if (addr < FLASH_USER_START_ADDR || addr > FLASH_USER_END_ADDR) {
        return FLASH_ERROR_ADDR;
    }
    return FLASH_OK;
}

/**
 * @brief 检查地址范围是否合法
 * @param addr 起始地址
 * @param len 长度
 * @return FLASH_OK 或 FLASH_ERROR_ADDR
 */
static flash_status_t flash_check_range(uint32_t addr, uint16_t len)
{
    if (addr < FLASH_USER_START_ADDR) {
        return FLASH_ERROR_ADDR;
    }
    if ((addr + len - 1) > FLASH_USER_END_ADDR) {
        return FLASH_ERROR_ADDR;
    }
    return FLASH_OK;
}

/**
 * @brief 解锁 Flash
 */
static flash_status_t flash_unlock(void)
{
    HAL_FLASH_Unlock();
    
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
        return FLASH_ERROR_UNLOCK;
    }
    return FLASH_OK;
}

/**
 * @brief 锁定 Flash
 */
static flash_status_t flash_lock(void)
{
    HAL_FLASH_Lock();
    
    if ((FLASH->CR & FLASH_CR_LOCK) == 0U) {
        return FLASH_ERROR_LOCK;
    }
    return FLASH_OK;
}

/**
 * @brief 获取地址所在的页号
 */
static uint32_t flash_get_page(uint32_t addr)
{
    return (addr - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
}

/**
 * @brief 擦除指定页
 * @param page_addr 页起始地址
 */
static flash_status_t flash_erase_page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = flash_get_page(page_addr);
    erase_init.NbPages = 1;
    
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        return FLASH_ERROR_ERASE;
    }
    
    /* 验证擦除结果 */
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 4) {
        if (*(__IO uint32_t *)(page_addr + i) != 0xFFFFFFFFU) {
            return FLASH_ERROR_ERASE;
        }
    }
    
    return FLASH_OK;
}

/**
 * @brief 双字编程 (64-bit)
 * @param addr 目标地址（必须8字节对齐）
 * @param data 64位数据
 */
static flash_status_t flash_program_doubleword(uint32_t addr, uint64_t data)
{
    /* 检查对齐 */
    if ((addr & 0x7U) != 0U) {
        return FLASH_ERROR_ADDR;
    }
    
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) != HAL_OK) {
        return FLASH_ERROR_PROGRAM;
    }
    
    return FLASH_OK;
}

/**
 * @brief 校验写入的数据
 */
static flash_status_t flash_verify(uint32_t addr, uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (*(__IO uint8_t *)(addr + i) != data[i]) {
            return FLASH_ERROR_VERIFY;
        }
    }
    return FLASH_OK;
}

/*==========================================
 *          对外接口实现
 *==========================================*/

/**
 * @brief 写入数据到 Flash 用户区
 * @param addr 目标地址（必须在 0x0803F000 ~ 0x0803FFFF 范围内）
 * @param data 源数据指针
 * @param datalen 数据长度（输入），实际写入长度（输出）
 * @note 自动处理擦除（整区擦除）和双字对齐
 */
void flash_write(uint32_t addr, uint8_t *data, uint16_t *datalen)
{
    flash_status_t status;
    uint16_t len;
    
    /* 参数检查 */
    if (data == NULL || datalen == NULL) {
        if (datalen != NULL) {
            *datalen = 0;
        }
        return;
    }
    
    len = *datalen;
    
    /* 地址和范围合法性检查 */
    if (flash_check_range(addr, len) != FLASH_OK) {
        *datalen = 0;
        return;
    }
    
    /* 长度检查 */
    if (len == 0 || len > FLASH_USER_SIZE) {
        *datalen = 0;
        return;
    }
    
    /* 解锁 Flash */
    status = flash_unlock();
    if (status != FLASH_OK) {
        *datalen = 0;
        return;
    }
    
    /* 擦除整个用户区（两页） */
    for (uint32_t page = 0; page < FLASH_USER_PAGE_COUNT; page++) {
        uint32_t page_addr = FLASH_USER_START_ADDR + (page * FLASH_PAGE_SIZE);
        status = flash_erase_page(page_addr);
        if (status != FLASH_OK) {
            flash_lock();
            *datalen = 0;
            return;
        }
    }
    
    /* 双字编程（8字节对齐） */
    uint16_t i = 0;
    while (i < len) {
        uint64_t doubleword = 0;
        uint8_t bytes_to_copy = (len - i) > 8 ? 8 : (len - i);
        
        /* 填充双字（小端序） */
        for (uint8_t j = 0; j < bytes_to_copy; j++) {
            doubleword |= ((uint64_t)data[i + j]) << (j * 8);
        }
        /* 剩余字节保持0xFF（擦除后的值） */
        for (uint8_t j = bytes_to_copy; j < 8; j++) {
            doubleword |= 0xFFULL << (j * 8);
        }
        
        status = flash_program_doubleword(addr + i, doubleword);
        if (status != FLASH_OK) {
            flash_lock();
            *datalen = i; /* 返回已写入的字节数 */
            return;
        }
        
        i += 8;
    }
    
    /* 锁定 Flash */
    flash_lock();
    
    /* 数据校验 */
    status = flash_verify(addr, data, len);
    if (status != FLASH_OK) {
        *datalen = 0;
        return;
    }
    
    *datalen = len;
}

/**
 * @brief 从 Flash 用户区读取数据
 * @param addr 源地址（必须在用户区范围内）
 * @param data 目标缓冲区
 * @param datalen 要读取的长度（输入），实际读取长度（输出）
 */
void flash_read(uint32_t addr, uint8_t *data, uint16_t *datalen)
{
    uint16_t len;
    
    /* 参数检查 */
    if (data == NULL || datalen == NULL) {
        if (datalen != NULL) {
            *datalen = 0;
        }
        return;
    }
    
    len = *datalen;
    
    /* 地址范围检查 */
    if (addr < FLASH_USER_START_ADDR || addr > FLASH_USER_END_ADDR) {
        *datalen = 0;
        return;
    }
    
    /* 检查读取范围是否越界 */
    if ((addr + len - 1) > FLASH_USER_END_ADDR) {
        len = FLASH_USER_END_ADDR - addr + 1;
    }
    
    if (len == 0) {
        *datalen = 0;
        return;
    }
    
    /* 直接内存拷贝 */
    memcpy(data, (void *)addr, len);
    
    *datalen = len;
}

/**
 * @brief 获取用户区起始地址
 * @return Flash 用户区起始地址 0x0803F000
 */
uint32_t flash_get_user_start_address(void)
{
    return FLASH_USER_START_ADDR;
}

/**
 * @brief 获取用户区结束地址
 * @return Flash 用户区结束地址 0x0803FFFF
 */
uint32_t flash_get_user_end_address(void)
{
    return FLASH_USER_END_ADDR;
}

/**
 * @brief 获取用户区总大小
 * @return 用户区大小 4096 字节 (4KB)
 */
uint32_t flash_get_user_size(void)
{
    return FLASH_USER_SIZE;
}

/**
 * @brief 获取 Flash 页大小
 * @return 页大小 2048 字节 (2KB)
 */
uint32_t flash_get_page_size(void)
{
    return FLASH_PAGE_SIZE;
}

/**
 * @brief 检查 Flash 区域是否为空（全0xFF）
 * @param addr 起始地址
 * @param len 检查长度
 * @return 1-为空，0-非空，-1-错误
 */
int8_t flash_is_empty(uint32_t addr, uint16_t len)
{
    if (addr < FLASH_USER_START_ADDR || (addr + len - 1) > FLASH_USER_END_ADDR) {
        return -1;
    }
    
    for (uint16_t i = 0; i < len; i++) {
        if (*(__IO uint8_t *)(addr + i) != 0xFF) {
            return 0;
        }
    }
    
    return 1;
}

/**
 * @brief 清空整个用户区（两页全部擦除为0xFF）
 * @return 0-成功，非0-失败
 */
uint8_t flash_clear_user_area(void)
{
    flash_status_t status;
    
    status = flash_unlock();
    if (status != FLASH_OK) {
        return 1;
    }
    
    /* 擦除两页 */
    for (uint32_t page = 0; page < FLASH_USER_PAGE_COUNT; page++) {
        uint32_t page_addr = FLASH_USER_START_ADDR + (page * FLASH_PAGE_SIZE);
        status = flash_erase_page(page_addr);
        if (status != FLASH_OK) {
            flash_lock();
            return 1;
        }
    }
    
    flash_lock();
    
    return 0;
}

/**
 * @brief 擦除指定页（在用户区内）
 * @param page_offset 页偏移（0=第一页，1=第二页）
 * @return 0-成功，非0-失败
 */
uint8_t flash_erase_user_page(uint8_t page_offset)
{
    flash_status_t status;
    
    if (page_offset >= FLASH_USER_PAGE_COUNT) {
        return 1;
    }
    
    uint32_t page_addr = FLASH_USER_START_ADDR + (page_offset * FLASH_PAGE_SIZE);
    
    status = flash_unlock();
    if (status != FLASH_OK) {
        return 1;
    }
    
    status = flash_erase_page(page_addr);
    
    flash_lock();
    
    return (status == FLASH_OK) ? 0 : 1;
}
