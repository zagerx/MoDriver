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

/* 控制周期 100us (10kHz) */
#define CONTROL_PERIOD_DT 0.0001f

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

void motor_bind_param_ext(struct motor *motor, struct motor_param_ext *param_ext)
{
	if (!motor || !param_ext || !param_ext->feedback_param) {
		return;
	}
	feedback_bind_encoder_param(motor->feedback, param_ext->feedback_param);
}

int16_t motor_param_check(struct motor *motor)
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
	(void)param_ext->crc_16; /* 暂时标记为已使用，避免警告 */
	return 0;                /* 参数检查通过 */
}

void motor_init(struct motor *motor)
{
	struct statemachine *sm = motor->sm;
	if (motor_param_check(motor)) {
		statemachine_init(sm, motor, motor_carib_state, NULL, 0);
	} else {
		statemachine_init(sm, motor, motor_idle_state, NULL, 0);
	}
}

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
