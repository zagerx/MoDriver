// SPDX-License-Identifier: GPL-2.0

#include "statemachine.h"
#include "_motorlib_internal.h"
#include "calibration.h"
#include "inverter.h"
#include "motor_state.h"

#include <stdint.h>

#include "motorlib_control_param.h"
#include "open_loop.h"
#include "close_loop.h"

void motor_openloop_encoder_state(struct statemachine *sm);

/**
 * @brief 电机校准状态
 *
 * @param sm 状态机实例
 *
 * 电机参数异常时进入的校准状态。
 * 调用 calibration_task 执行校准，校准模块内部直接操作逆变器。
 */
void motor_carib_state(struct statemachine *sm)
{
	enum {
		CALIBRATING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	enum calibration_status calib_status;

	switch (sm->phase) {
	case ENTER:
		/* 进入校准状态 */
		calibration_init(motor);
		sm->phase = CALIBRATING;
		break;

	case CALIBRATING:
		/* 调用校准任务，由校准模块自主控制逆变器等硬件 */
		calib_status = calibration_task(motor);

		/* 根据校准结果迁移状态 */
		if (calib_status == CALIBRATION_STATUS_SUCCESS) {
			TRAN_STATE(sm, motor_runing_state);
		} else if (calib_status == CALIBRATION_STATUS_FAILED) {
			/* 校准失败 */
			TRAN_STATE(sm, motor_init_state);
		}
		break;

	case EXIT:
		/* 退出校准状态，确保逆变器禁用（安全考虑） */
		/* 注：即使校准模块内部已禁用，这里再禁一次确保万无一失 */
		if (motor->inverter) {
			inverter_disable(motor->inverter);
		}
		break;

	default:
		break;
	}
}

/**
 * @brief 电机初始化状态
 *
 * @param sm 状态机实例
 *
 * 电机初始化后的默认状态，等待运行指令。
 */
void motor_init_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;

	case RUNING:
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief 电机空闲状态
 *
 * @param sm 状态机实例
 *
 * 电机初始化后的默认状态，等待运行指令。
 */
void motor_idle_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;

	case RUNING:
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief 电机运行状态
 *
 * @param sm 状态机实例
 *
 * 电机正常运行状态，执行FOC控制循环。
 */
void motor_runing_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = motor->inverter;

	switch (sm->phase) {
	case ENTER:
		inverter_enable(inverter);
		sm->phase = RUNING;
		sm->count = 0;
		break;

	case RUNING:
		if (sm->count++ > SPEED_LOOP_INTERVAL) {
			sm->count = 0;

			motor_velocity_loop(motor);
		}
		motor_currment_loop(motor);
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief 电机开环编码器状态
 *
 * @param sm 状态机实例
 *
 * 电机开环编码器测试状态。
 */
void motor_openloop_encoder_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = motor->inverter;
	static float target;
	// struct feedback *feedback = motor->feedback;
	// static uint32_t debug_cnt = 0;

	switch (sm->phase) {
	case ENTER:
		inverter_enable(inverter);
		target = 1.0f;
		sm->phase = RUNING;
		break;

	case RUNING:
		target = motor->param_ext->foc_param.target_pos;
		open_loop_encoder(motor, target);
		// if (++debug_cnt % 500 == 0) {
		// 	debug_cnt = 0;
		// 	target = -target;                            // 反转目标位置
		// 	foc_pid_reset(&motor->foc.data.ctrl.d_axis); // 重置PID控制器状态
		// }
		// currment_debug(motor, target);
		break;

	case EXIT:
		inverter_disable(inverter);
		break;

	default:
		break;
	}
}
