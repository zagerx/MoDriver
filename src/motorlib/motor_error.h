/* SPDX-License-Identifier: GPL-2.0 */

#ifndef MOTOR_ERROR_H
#define MOTOR_ERROR_H

#include <stdint.h>

/**
 * @brief 将错误位枚举转换为位掩码
 * @param x 枚举值（从1开始）
 * @return 对应的位掩码（1 << (x-1)）
 */
#define MOTOR_ERR_BIT(x) (1U << ((x) - 1))

/**
 * @brief 电机错误码位定义（按位组合）
 * @note 配合 MOTOR_ERR_BIT() 宏使用，error_code 为 uint32_t 类型
 * @note 枚举值从1开始自动递增，实际位掩码 = 1 << (enum_value - 1)
 */
enum motor_error {
	MOTOR_ERR_PARAM_INVALID = 1, /**< @brief 参数无效 */
	MOTOR_ERR_PARAM_CRC,         /**< @brief CRC校验失败 */
	MOTOR_ERR_PARAM_RANGE,       /**< @brief 参数越界 */
	MOTOR_ERR_RUN_OVER_CURR,     /**< @brief 过流 */
	MOTOR_ERR_RUN_OVER_SPEED,    /**< @brief 超速 */
	MOTOR_ERR_RUN_FOLLOW,        /**< @brief 跟随误差过大 */
	MOTOR_ERR_SYS_INIT,          /**< @brief 初始化失败 */
	MOTOR_ERR_SYS_STATE,         /**< @brief 状态机异常 */
	MOTOR_ERR_SYS_TIMEOUT,       /**< @brief 超时 */
	MOTOR_ERR_CRITICAL,          /**< @brief 严重错误，需停机 */
};

/**
 * @brief 错误码操作宏
 * @note 所有操作均通过 MOTOR_ERR_BIT() 转换枚举值为位掩码
 * @{
 */

/** @brief 设置错误位 */
#define MOTOR_ERR_SET(err, bit) ((err) |= MOTOR_ERR_BIT(bit))

/** @brief 清除错误位 */
#define MOTOR_ERR_CLEAR(err, bit) ((err) &= ~MOTOR_ERR_BIT(bit))

/** @brief 测试错误位是否置位 */
#define MOTOR_ERR_TEST(err, bit) ((err) & MOTOR_ERR_BIT(bit))

/** @brief 是否有任何错误 */
#define MOTOR_ERR_ANY(err) ((err) != 0)

/** @brief 是否无错误 */
#define MOTOR_ERR_NONE(err) ((err) == 0)

/** @brief 清除所有错误 */
#define MOTOR_ERR_CLEAR_ALL(err) ((err) = 0)

/** @} */

#endif /* MOTOR_ERROR_H */
