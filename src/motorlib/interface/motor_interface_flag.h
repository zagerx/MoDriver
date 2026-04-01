
/**
 * @file motor_driver.h
 * @brief 电机驱动公共接口头文件
 * @details 定义硬件操作接口、参数结构体及状态/模式枚举，供Hardware层与motorlib交互使用
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief 电机状态标志位定义（按位组合）
 * @note 低16位为动态运行状态，高16位为事件/完成/持久标志
 * @note 配合 MOTOR_STATUS_BIT() 宏使用，status_flag 为 uint32_t 类型
 */
enum motor_flag_bits {

	MOTOR_FLAGS_ENABLED = 0,             /**< @brief 逆变器使能 */
	MOTOR_FLAGS_HOMING = 1,              /**< @brief 原点回归中 */
	MOTOR_FLAGS_POSITION_LATCH = 2,      /**< @brief 位置锁存中 */
	MOTOR_FLAGS_TARGET_REACHED = 3,      /**< @brief 目标已到达 */
	MOTOR_FLAGS_MOVING = 4,              /**< @brief 电机正在运动 */
	MOTOR_FLAGS_HOMING_DONE = 5,         /**< @brief 原点回归完成 */
	MOTOR_FLAGS_CALIBRATION_DONE = 6,    /**< @brief 校准完成 */
	MOTOR_FLAGS_SPEED_ZERO = 7,          /**< @brief 速度接近零 */
	MOTOR_FLAGS_POSITION_LATCH_DONE = 8, /**< @brief 位置锁存完成 */
	MOTOR_FLAGS_TRAJECTORY_BUSY = 9,     /**< @brief 轨迹规划器运行中 */
};

enum motor_errorcode_bits {
	MOTOR_ERROR_NONE = 0,         /**< @brief 无错误 */
	MOTOR_ERROR_HW_FAILURE = 1,   /**< @brief 硬件故障 */
	MOTOR_ERROR_PARAM = 2,        /**< @brief 参数错误 */
	MOTOR_ERROR_OVERVOLTAGE = 3,  /**< @brief 过压保护触发 */
	MOTOR_ERROR_UNDERVOLTAGE = 4, /**< @brief 欠压保护触发 */
	MOTOR_ERROR_OVERCURRENT = 5,  /**< @brief 过流保护触发 */
	MOTOR_ERROR_OVERTEMP = 6,     /**< @brief 过热保护触发 */
};

enum motor_status {
	MOTOR_STATUS_INIT = 0,
	MOTOR_STATUS_CALIB,
	MOTOR_STATUS_IDLE,
	MOTOR_STATUS_RUNING,
	MOTOR_STATUS_MAX,
};
enum motor_mode {
	/* 标准未定义/无模式 */
	MODE_NONE = 0x00, /*!< 无模式分配 (Not assigned) */

	/* 轮廓模式 (Profile Modes) - 非同步，内部规划轨迹 */
	MODE_PP = 0x01, /*!< 轮廓位置模式 (Profile Position Mode) */
	MODE_PV = 0x03, /*!< 轮廓速度模式 (Profile Velocity Mode) */
	MODE_PT = 0x04, /*!< 轮廓转矩模式 (Profile Torque Mode) - 较少使用 */
	MODE_HM = 0x06, /*!< 原点回归模式 (Homing Mode) */

	/* 循环同步模式 (Cyclic Synchronous Modes) - 同步，主站规划轨迹 */
	MODE_IP = 0x07,  /*!< 插补位置模式 (Interpolated Position Mode) */
	MODE_CSP = 0x08, /*!< 循环同步位置模式 (Cyclic Synchronous Position Mode) */
	MODE_CSV = 0x09, /*!< 循环同步速度模式 (Cyclic Synchronous Velocity Mode) */
	MODE_CST = 0x0A, /*!< 循环同步转矩模式 (Cyclic Synchronous Torque Mode) */

	/* 扩展模式 */
	MODE_CSTCA = 0x0B, /*!< 循环同步转矩带通讯角模式 (CST with Commutation Angle) */

	/* 制造商特定 (0x7F-0xFF 保留) */
	MODE_MANUFACTURER = 0xFF, /*!< 制造商特定模式 */
};

#endif /* MOTOR_DRIVER_H */
