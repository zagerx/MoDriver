
/**
 * @file motor_bits.h
 * @brief motorlib 内部标志位与错误码操作宏
 * @details 定义 MOTOR_FLAGS_xxx 和 MOTOR_ERR_xxx 操作宏，仅供 motorlib 内部模块使用
 * @note 此文件为内部头文件，外部模块不应直接包含和使用
 */

#ifndef MOTOR_BITS_H
#define MOTOR_BITS_H

/**
 * @note 本头文件依赖 motor_interface_flag.h 和 motor_interface_mode.h 中的枚举定义，
 *       包含时请确保前置包含对应头文件
 */

/**
 * @brief 状态标志位操作宏
 * @note 标志枚举值本身即为位掩码，无需额外转换
 * @{
 */

/** @brief 设置状态位 */
#define MOTOR_FLAGS_SET(flags, bit) ((flags) |= (bit))

/** @brief 清除状态位 */
#define MOTOR_FLAGS_CLEAR(flags, bit) ((flags) &= ~(bit))

/** @brief 测试状态位是否置位 */
#define MOTOR_FLAGS_TEST(flags, bit) ((flags) & (bit))

/** @brief 是否有任何状态标志 */
#define MOTOR_FLAGS_ANY(flags) ((flags) != 0)

/** @brief 是否无任何状态标志 */
#define MOTOR_FLAGS_NONE(flags) ((flags) == 0)

/** @brief 清除所有状态标志 */
#define MOTOR_FLAGS_CLEAR_ALL(flags) ((flags) = 0)

/** @} */

/**
 * @brief 错误码操作宏
 * @note 错误枚举值本身即为位掩码，无需额外转换
 * @{
 */

/** @brief 设置错误位 */
#define MOTOR_ERR_SET(err, bit) ((err) |= (bit))

/** @brief 清除错误位 */
#define MOTOR_ERR_CLEAR(err, bit) ((err) &= ~(bit))

/** @brief 测试错误位是否置位 */
#define MOTOR_ERR_TEST(err, bit) ((err) & (bit))

/** @brief 是否有任何错误 */
#define MOTOR_ERR_ANY(err) ((err) != 0)

/** @brief 是否无错误 */
#define MOTOR_ERR_NONE(err) ((err) == 0)

/** @brief 清除所有错误 */
#define MOTOR_ERR_CLEAR_ALL(err) ((err) = 0)

/** @} */

#endif /* MOTOR_BITS_H */
