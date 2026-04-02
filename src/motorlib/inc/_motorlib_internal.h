
/**
 * @file _motorlib_internal.h
 * @brief motorlib 内部头文件
 * @details 定义电机实例结构体、数据结构体及配置结构体，仅供 motorlib 内部模块使用
 * @note 此文件为内部头文件，外部模块不应直接包含和使用
 */

#ifndef _MOTOR_INTERNAL_H
#define _MOTOR_INTERNAL_H

#include "motor_interface.h"
#include "foc.h"
#include "calibration.h"
#include "motor_interface_flag.h"
#include "trajectory_plan.h"
#include "statemachine.h"
#include "motor_protection.h"
struct inverter;
struct feedback;
struct currsmp;

/**
 * @brief 电机数据结构体
 */
struct motor_data {
	uint32_t flags;    /**< @brief 状态标志位组合值（位掩码） */
	uint32_t errorcode; /**< @brief 错误码位组合值（位掩码） */
};

/**
 * @brief 电机配置结构体
 */
struct motor_config {
	uint16_t pairs; /**< @brief 极对数 */
};

/**
 * @brief 电机实例结构体
 */
struct motor {
	struct motor_param_ext *param_ext;  /**< @brief 扩展参数 */
	struct inverter *inverter;          /**< @brief 逆变器实例 */
	struct feedback *feedback;          /**< @brief 反馈实例 */
	struct currsmp *currsmp;            /**< @brief 电流采样实例 */
	struct statemachine sm;             /**< @brief 状态机实例 */
	struct statemachine sm_mode;        /**< @brief 模式状态机实例 */
	struct motor_data data;             /**< @brief 数据 */
	struct motor_config config;         /**< @brief 配置 */
	struct calibration calib;           /**< @brief 校准实例 */
	struct trajectory_plan traj_plan;   /**< @brief 轨迹规划实例 */
	struct foc foc;                     /**< @brief FOC相关数据 */
	struct protection_manager prot_mgr; /**< @brief 保护模块实例 */
};

#endif /* _MOTOR_INTERNAL_H */
