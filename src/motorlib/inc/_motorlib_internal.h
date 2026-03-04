#ifndef _MOTOR_INTERNAL_H
#define _MOTOR_INTERNAL_H

#include "motor_driver.h"
#include "motor_error.h"
#include "foc.h"
struct statemachine;
struct inverter;
struct feedback;

/**
 * @brief 电机数据结构体
 */
struct motor_data {
	uint32_t error_code; /**< @brief 错误码（位组合，使用 enum motor_error_bits +
				MOTOR_ERR_BIT()） */
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
	struct inverter *inverter; /**< @brief 逆变器实例 */
	struct feedback *feedback; /**< @brief 反馈实例 */
	struct statemachine *sm;   /**< @brief 状态机实例 */

	struct foc foc;                    /**< @brief FOC相关数据 */
	struct motor_data *data;           /**< @brief 数据 */
	struct motor_config *config;       /**< @brief 配置 */
	struct motor_param_ext *param_ext; /**< @brief 扩展参数 */
};

#endif
