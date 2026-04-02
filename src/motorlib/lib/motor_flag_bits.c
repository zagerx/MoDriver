
/**
 * @file motor_flag_bits.c
 * @brief 电机状态标志位相关实现
 * @details 提供外部只读访问 motor_data.flags 的 Getter API
 */

#include "_motorlib_internal.h"
/**
 * @brief 将状态标志位枚举转换为位掩码
 * @param x 枚举值
 * @return 对应的位掩码（1U << x）
 */
#define MOTOR_FLAGS_BIT(x) (1U << (x))

/**
 * @brief 状态标志位操作宏
 * @note 所有操作均通过 MOTOR_FLAGS_BIT() 转换枚举值为位掩码
 * @{
 */

/** @brief 设置状态位 */
#define MOTOR_FLAGS_SET(flags, bit) ((flags) |= MOTOR_FLAGS_BIT(bit))

/** @brief 清除状态位 */
#define MOTOR_FLAGS_CLEAR(flags, bit) ((flags) &= ~MOTOR_FLAGS_BIT(bit))

/** @brief 测试状态位是否置位 */
#define MOTOR_FLAGS_TEST(flags, bit) ((flags) & MOTOR_FLAGS_BIT(bit))

/** @brief 是否有任何状态标志 */
#define MOTOR_FLAGS_ANY(flags) ((flags) != 0)

/** @brief 是否无任何状态标志 */
#define MOTOR_FLAGS_NONE(flags) ((flags) == 0)

/** @brief 清除所有状态标志 */
#define MOTOR_FLAGS_CLEAR_ALL(flags) ((flags) = 0)
/**
 * @brief 获取电机当前状态标志位
 * @param[in] motor 电机实例
 * @return uint32_t 状态标志位组合值
 * @note 对应 motor_data.status_flag，外部只读获取
 */
uint32_t motor_get_flag(const struct motor *motor)
{
	return motor ? motor->data.flags : 0;
}

/**
 * @brief 检查指定状态标志位是否置位
 * @param[in] motor 电机实例
 * @param[in] bit 状态标志位枚举值
 * @return int 1 表示置位，0 表示未置位或实例为空
 */
int motor_is_flag_set(const struct motor *motor, enum motor_flag_bits bit)
{
	return motor ? MOTOR_FLAGS_TEST(motor->data.flags, bit) != 0 : 0;
}

/**
 * @brief 获取电机当前错误码位组合值
 * @param[in] motor 电机实例
 * @return uint32_t 错误码位组合值
 */
uint32_t motor_get_errorcode(const struct motor *motor)
{
	return motor ? motor->data.errorcode : 0;
}

/**
 * @brief 检查指定错误码位是否置位
 * @param[in] motor 电机实例
 * @param[in] bit 错误码位枚举值
 * @return int 1 表示置位，0 表示未置位或实例为空
 */
int motor_is_error_set(const struct motor *motor, enum motor_errorcode_bits bit)
{
	return motor ? MOTOR_ERR_TEST(motor->data.errorcode, bit) != 0 : 0;
}
