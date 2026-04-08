
/**
 * @file _motorlib_internal.h
 * @brief motorlib 内部头文件
 * @details 定义电机实例结构体、数据结构体及配置结构体，仅供 motorlib 内部模块使用
 * @note 此文件为内部头文件，外部模块不应直接包含和使用
 */

#ifndef _MOTOR_INTERNAL_H
#define _MOTOR_INTERNAL_H

#include <stdint.h>
#include "motor_interface_bits.h"
#include "motor_interface_mode.h"
#include "foc.h"
#include "calibration.h"
#include "trajectory_plan.h"
#include "statemachine.h"
#include "motor_protection.h"

struct motor_param_ext;
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
// struct inverter;
// struct feedback;
// struct currsmp;

/**
 * @brief 调试数据开关，默认开启
 */
#ifndef MOTORLIB_DEBUG_ENABLED
#define MOTORLIB_DEBUG_ENABLED 1
#endif

#if MOTORLIB_DEBUG_ENABLED
/**
 * @brief 电机调试数据结构体
 */
struct motor_debug {
	volatile float test_tar_pos;   /**< @brief 调试：目标位置 */
	volatile float test_real_pos;  /**< @brief 调试：实际位置 */
	volatile float test_plann_vel; /**< @brief 调试：规划速度 */
	uint16_t test_flag1;           /**< @brief 调试：标志1 */
	uint16_t test_flag2;           /**< @brief 调试：标志2 */
};
#endif

/**
 * @brief 电机数据结构体
 */
struct motor_data {
	uint32_t flags;        /**< @brief 状态标志位组合值（位掩码） */
	uint32_t errorcode;    /**< @brief 错误码位组合值（位掩码） */
	uint32_t command;      /**< @brief 命令位组合值（位掩码） */
	float openloop_target; /**< @brief 开环编码器目标值 */
#if MOTORLIB_DEBUG_ENABLED
	struct motor_debug debug; /**< @brief 调试数据 */
#endif
};

/**
 * @brief 电机实例结构体
 */
struct motor {
	struct motor_param_ext *param_ext;  /**< @brief 扩展参数 */
	struct inverter inverter;           /**< @brief 逆变器实例 */
	struct feedback feedback;           /**< @brief 反馈实例 */
	struct currsmp currsmp;             /**< @brief 电流采样实例 */
	struct statemachine sm;             /**< @brief 状态机实例 */
	struct statemachine sm_mode;        /**< @brief 模式状态机实例 */
	struct motor_data data;             /**< @brief 数据 */
	struct calibration calib;           /**< @brief 校准实例 */
	struct trajectory_plan traj_plan;   /**< @brief 轨迹规划实例 */
	struct foc foc;                     /**< @brief FOC相关数据 */
	struct protection_manager prot_mgr; /**< @brief 保护模块实例 */
};

#endif /* _MOTOR_INTERNAL_H */
