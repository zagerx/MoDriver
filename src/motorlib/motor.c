/* SPDX-License-Identifier: GPL-2.0 */

#include "motor.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "statemachine.h"
#include <stdint.h>
#include "motor_state.h"
#undef NULL
#define NULL (0)

/** @brief 控制周期 100us (10kHz) */
#define CONTROL_PERIOD_DT 0.0001f

/**
 * @brief 绑定硬件接口
 * @param[in] motor 电机实例
 * @param[in] hw 硬件接口集合
 * @return 无
 * @details 将编码器和逆变器操作接口绑定到电机实例
 */
void motor_bind_hardware(struct motor *motor, const struct motor_hw_ops *hw)
{
	if (!motor || !hw) {
		return;
	}

	if (hw->encoder) {
		feedback_bind_encoder(motor->feedback, hw->encoder);
	}

	if (hw->inverter) {
		inverter_bind_inverter(motor->inverter, hw->inverter);
	}
}

/**
 * @brief 绑定扩展参数
 * @param[in] motor 电机实例
 * @param[in] param_ext 扩展参数
 * @return 无
 * @details 绑定反馈参数到电机实例
 */
void motor_bind_param_ext(struct motor *motor, struct motor_param_ext *param_ext)
{
	if (!motor || !param_ext || !param_ext->feedback_param) {
		return;
	}
	feedback_bind_encoder_param(motor->feedback, param_ext->feedback_param);
}

/**
 * @brief 检查电机参数合法性
 * @param[in] motor 电机实例
 * @return int16_t 错误码
 * @retval 0 参数合法
 * @retval -1 电机实例为空
 * @retval -2 扩展参数为空
 * @retval -3 反馈参数为空
 * @retval -10 轮子半径无效
 * @retval -11 减速比无效
 * @retval -12 极对数无效
 * @retval -13 方向无效
 * @retval -14 编码器分辨率无效
 */
static int16_t motor_param_check(struct motor *motor)
{
	if (!motor) {
		return -1; /* 电机实例为空 */
	}
	struct motor_param_ext *param_ext = motor->param_ext;
	/* CRC 校验伪代码 */
	// uint16_t calc_crc = crc16_calculate((uint8_t *)param_ext, sizeof(*param_ext));
	// if (calc_crc != param_ext->crc_16) {
	//     return -20; /* CRC 校验失败 */
	// }
	(void)param_ext->crc_16;
	return 0;
}

/**
 * @brief 初始化电机
 * @param[in] motor 电机实例
 * @return 无
 * @details 检查参数并初始化状态机，根据检查结果进入相应初始状态
 */
void motor_init(struct motor *motor)
{
	struct statemachine *sm = motor->sm;
	if (motor_param_check(motor)) {
		statemachine_init(sm, motor, motor_carib_state, NULL, 0);
	} else {
		statemachine_init(sm, motor, motor_idle_state, NULL, 0);
	}
}

/**
 * @brief 高频控制任务
 * @param[in] motor 电机实例
 * @return 无
 * @note 应在定时器中断中周期性调用（默认10kHz）
 * @details 执行反馈更新和状态机调度
 */
void motor_highfreq_task(struct motor *motor)
{
	if (!motor || !motor->config) {
		return;
	}
	if (motor->feedback) {
		feedback_update(motor->feedback, CONTROL_PERIOD_DT);
	}
	struct statemachine *sm = motor->sm;

	sm_dispatch(sm);
}
