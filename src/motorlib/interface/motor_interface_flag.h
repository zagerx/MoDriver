
/**
 * @file motor_interface_flag.h
 * @brief 电机驱动公共接口头文件
 * @details 定义硬件操作接口、参数结构体及状态/模式枚举，供Hardware层与motorlib交互使用
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief 电机状态标志位定义（按位组合）
 * @note 低16位为动态运行状态，高16位为事件/完成/持久标志
 * @note 枚举值本身即为位掩码，status_flag 为 uint32_t 类型
 */
enum motor_flag_bits {
	MOTOR_FLAGS_ENABLED = 1U << 0,             /**< @brief 逆变器使能 */
	MOTOR_FLAGS_HOMING = 1U << 1,              /**< @brief 原点回归中 */
	MOTOR_FLAGS_POSITION_LATCH = 1U << 2,      /**< @brief 位置锁存中 */
	MOTOR_FLAGS_TARGET_REACHED = 1U << 3,      /**< @brief 目标已到达 */
	MOTOR_FLAGS_MOVING = 1U << 4,              /**< @brief 电机正在运动 */
	MOTOR_FLAGS_HOMING_DONE = 1U << 5,         /**< @brief 原点回归完成 */
	MOTOR_FLAGS_CALIBRATION_DONE = 1U << 6,    /**< @brief 校准完成 */
	MOTOR_FLAGS_SPEED_ZERO = 1U << 7,          /**< @brief 速度接近零 */
	MOTOR_FLAGS_POSITION_LATCH_DONE = 1U << 8, /**< @brief 位置锁存完成 */
	MOTOR_FLAGS_TRAJECTORY_BUSY = 1U << 9,     /**< @brief 轨迹规划器运行中 */
};
enum motor_errorcode_bits {
	MOTOR_ERROR_NONE = 0U,              /**< @brief 无错误 */
	MOTOR_ERROR_HW_FAILURE = 1U << 0,   /**< @brief 硬件故障 */
	MOTOR_ERROR_PARAM = 1U << 1,        /**< @brief 参数错误 */
	MOTOR_ERROR_OVERVOLTAGE = 1U << 2,  /**< @brief 过压保护触发 */
	MOTOR_ERROR_UNDERVOLTAGE = 1U << 3, /**< @brief 欠压保护触发 */
	MOTOR_ERROR_OVERCURRENT = 1U << 4,  /**< @brief 过流保护触发 */
	MOTOR_ERROR_OVERTEMP = 1U << 5,     /**< @brief 过热保护触发 */
	MOTOR_ERROR_UNDERTEMP = 1U << 6,    /**< @brief 欠热保护触发 */
	MOTOR_ERROR_STALL = 1U << 7,        /**< @brief 堵转保护触发 */
};
#endif /* MOTOR_DRIVER_H */
