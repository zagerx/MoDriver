
/**
 * @file motor_interface_bits.h
 * @brief 电机位标志公共接口头文件
 * @details 定义电机状态标志位、错误码、命令位的枚举及操作API
 *          供外部模块查询电机状态和发送控制命令
 */

#ifndef MOTOR_INTERFACE_BITS_H
#define MOTOR_INTERFACE_BITS_H

#include <stdint.h>

/**
 * @brief 电机状态标志位定义（按位组合）
 * @note 所有位均定义在低16位，包含动态运行状态和事件/完成标志的混合
 * @note 枚举值本身即为位掩码，对应 motor_data.flags (uint32_t)
 */
enum motor_flag_bits
{
	MOTOR_FLAGS_ENABLED = 1U << 0,		   /**< @brief 逆变器使能 */
	MOTOR_FLAGS_POSITION_LATCH = 1U << 2,	   /**< @brief 位置锁存中 */
	MOTOR_FLAGS_TARGET_REACHED = 1U << 3,	   /**< @brief 目标已到达 */
	MOTOR_FLAGS_MOVING = 1U << 4,		   /**< @brief 电机正在运动 */
	MOTOR_FLAGS_HOMING_DONE = 1U << 5,	   /**< @brief 原点回归完成 */
	MOTOR_FLAGS_CALIBRATION_DONE = 1U << 6,	   /**< @brief 校准完成 */
	MOTOR_FLAGS_SPEED_ZERO = 1U << 7,	   /**< @brief 速度接近零 */
	MOTOR_FLAGS_POSITION_LATCH_DONE = 1U << 8, /**< @brief 位置锁存完成 */
	MOTOR_FLAGS_TRAJECTORY_BUSY = 1U << 9,	   /**< @brief 轨迹规划器运行中 */
};

/**
 * @brief 电机错误码位定义（按位组合）
 * @note 对应 motor_data.errorcode (uint32_t)
 */
enum motor_errorcode_bits
{
	MOTOR_ERROR_NONE = 0U,		    /**< @brief 无错误 */
	MOTOR_ERROR_HW_FAILURE = 1U << 0,   /**< @brief 硬件故障 */
	MOTOR_ERROR_PARAM = 1U << 1,	    /**< @brief 参数错误 */
	MOTOR_ERROR_OVERVOLTAGE = 1U << 2,  /**< @brief 过压保护触发 */
	MOTOR_ERROR_UNDERVOLTAGE = 1U << 3, /**< @brief 欠压保护触发 */
	MOTOR_ERROR_OVERCURRENT = 1U << 4,  /**< @brief 过流保护触发 */
	MOTOR_ERROR_OVERTEMP = 1U << 5,	    /**< @brief 过热保护触发 */
	MOTOR_ERROR_UNDERTEMP = 1U << 6,    /**< @brief 欠热保护触发 */
	MOTOR_ERROR_STALL = 1U << 7,	    /**< @brief 堵转保护触发 */
};

/**
 * @brief 电机命令位定义（按位组合）
 * @note 对应 motor_data.command (uint32_t)
 */
enum motor_command_bits
{
	MOTOR_CMD_NONE = 0U,	    /**< @brief 无命令 */
	MOTOR_CMD_STOP = 1U << 0,   /**< @brief 停止运动 */
	MOTOR_CMD_RESET = 1U << 1,  /**< @brief 重置状态 */
	MOTOR_CMD_HOMING = 1U << 2, /**< @brief 执行原点回归 */
	MOTOR_CMD_CALIB = 1U << 3,  /**< @brief 执行校准 */
};

struct motor;

/* ==================== 状态标志位 API ==================== */

/**
 * @brief 设置电机状态标志位
 * @param[in] motor 电机实例
 * @param[in] bit 状态标志位枚举值
 * @return int 1 表示设置成功，0 表示失败或实例为空
 */
int motor_set_flag(struct motor *motor, enum motor_flag_bits bit);

/**
 * @brief 获取电机当前状态标志位组合值
 * @param[in] motor 电机实例
 * @return uint32_t 状态标志位组合值
 */
uint32_t motor_get_flag(const struct motor *motor);

/**
 * @brief 检查指定状态标志位是否置位
 * @param[in] motor 电机实例
 * @param[in] bit 状态标志位枚举值
 * @return int 1 表示置位，0 表示未置位或实例为空
 */
int motor_is_flag_set(const struct motor *motor, enum motor_flag_bits bit);

/**
 * @brief 清除指定状态标志位
 * @param[in] motor 电机实例
 * @param[in] bit 状态标志位枚举值
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_flag(struct motor *motor, enum motor_flag_bits bit);

/**
 * @brief 清除所有状态标志位
 * @param[in] motor 电机实例
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_all_flags(struct motor *motor);

/* ==================== 错误码 API ==================== */

/**
 * @brief 设置电机错误码位
 * @param[in] motor 电机实例
 * @param[in] bit 错误码位枚举值
 * @return int 1 表示设置成功，0 表示失败或实例为空
 */
int motor_set_error(struct motor *motor, enum motor_errorcode_bits bit);

/**
 * @brief 获取电机当前错误码位组合值
 * @param[in] motor 电机实例
 * @return uint32_t 错误码位组合值
 */
uint32_t motor_get_errorcode(const struct motor *motor);

/**
 * @brief 检查指定错误码位是否置位
 * @param[in] motor 电机实例
 * @param[in] bit 错误码位枚举值
 * @return int 1 表示置位，0 表示未置位或实例为空
 */
int motor_is_error_set(const struct motor *motor, enum motor_errorcode_bits bit);

/**
 * @brief 清除指定错误码位
 * @param[in] motor 电机实例
 * @param[in] bit 错误码位枚举值
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_error(struct motor *motor, enum motor_errorcode_bits bit);

/**
 * @brief 清除所有错误码
 * @param[in] motor 电机实例
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_all_errors(struct motor *motor);

/* ==================== 命令位 API ==================== */

/**
 * @brief 设置指定命令位
 * @param[in] motor 电机实例
 * @param[in] bit 命令位枚举值
 * @return int 1 表示置位成功，0 表示失败或实例为空
 */
int motor_set_command(struct motor *motor, enum motor_command_bits bit);

/**
 * @brief 清除指定命令位
 * @param[in] motor 电机实例
 * @param[in] bit 命令位枚举值
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_command(struct motor *motor, enum motor_command_bits bit);

/**
 * @brief 检查指定命令位是否置位
 * @param[in] motor 电机实例
 * @param[in] bit 命令位枚举值
 * @return int 1 表示置位，0 表示未置位或实例为空
 */
int motor_is_command_set(const struct motor *motor, enum motor_command_bits bit);

/**
 * @brief 清除所有命令位
 * @param[in] motor 电机实例
 * @return int 1 表示清除成功，0 表示失败或实例为空
 */
int motor_clear_all_commands(struct motor *motor);

#endif /* MOTOR_INTERFACE_BITS_H */
