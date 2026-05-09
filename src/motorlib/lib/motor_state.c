/**
 * @file motor_state.c
 * @brief 电机主状态机实现
 * @details 实现电机的初始化、校准、空闲、运行及开环编码器测试等主状态处理
 */

#include "statemachine.h"
#include "_motorlib_internal.h"
#include "calibration.h"
#include "inverter.h"
#include "motor_state.h"
#include "motor_mode.h"
#include <stdint.h>

#include "open_loop.h"
#include "close_loop.h"

/** @brief 电机开环编码器测试状态（前向声明） */
void motor_openloop_encoder_state(struct statemachine *sm);

/**
 * @brief 电机校准状态
 *
 * @param sm 状态机实例
 *
 * 电机参数异常时进入的校准状态。
 * 调用 calibration_task 执行校准，校准模块内部直接操作逆变器。
 */
void motor_calib_state(struct statemachine *sm)
{
	enum {
		CALIBRATING = USER_STATUS,
		CALIBRA_FALUT_FAIL,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = &motor->inverter;
	bool calib_done;

	switch (sm->phase) {
	case ENTER:
		/* 进入校准状态 */
		calibration_init(motor);
		motor->param_ext->is_calibrated = 0; /* 重置校准标志 */
		sm->phase = CALIBRATING;
		break;

	case CALIBRATING:
		/* 调用校准任务，由校准模块自主控制逆变器等硬件 */
		calib_done = calibration_task(motor);

		/* 根据校准结果迁移状态 */
		if (calib_done) {
			if (motor->calib.state == CAL_STATE_SUCCESS) {
				struct feedback *feedback = &motor->feedback;
				feedback_reset_encoder(feedback);
				motor->param_ext->is_calibrated = 1; /* 标记校准完成 */
				sm_transition(sm, motor_idle_state);
			} else {
				/* 校准失败 */
				sm->phase = CALIBRA_FALUT_FAIL;
			}
		}
		break;
	case CALIBRA_FALUT_FAIL:
		break;
	case EXIT:
		/* 退出校准状态，确保逆变器禁用（安全考虑） */
		/* 注：即使校准模块内部已禁用，这里再禁一次确保万无一失 */
		if (inverter) {
			inverter_disable(inverter);
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
		RUNNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNNING;
		break;

	case RUNNING:
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
		RUNNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNNING;
		break;

	case RUNNING:
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
		RUNNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = &motor->inverter;
	struct statemachine *sm_mode = &motor->sm_mode;
	switch (sm->phase) {
	case ENTER:
		inverter_set_voltage(inverter, 0.0f, 0.0f, 0.0f);
		inverter_enable(inverter);

		sm_transition_sync(sm_mode, motor_mode_none);
		sm->phase = RUNNING;
		sm->count = 0;
		break;

	case RUNNING:
		sm_dispatch(sm_mode);
		break;

	case EXIT:
		sm_transition_sync(sm_mode, motor_mode_none);
		break;

	default:
		break;
	}
}

/**
 * @brief 获取电机当前主状态
 * @param[in] motor 电机实例指针
 * @return enum motor_status 当前主状态
 * @details 通过状态机 current_state 反查枚举值
 */
enum motor_status motor_get_status(const struct motor *motor)
{
	if (!motor) {
		return MOTOR_STATUS_INIT;
	}

	sm_state_t state = motor->sm.current_state;
	if (state == motor_runing_state) {
		return MOTOR_STATUS_RUNNING;
	}
	if (state == motor_idle_state) {
		return MOTOR_STATUS_IDLE;
	}
	if (state == motor_calib_state) {
		return MOTOR_STATUS_CALIB;
	}
	return MOTOR_STATUS_INIT;
}

/**
 * @brief 切换电机主状态
 * @param[in] motor 电机实例指针
 * @param[in] new_state 新的电机状态
 * @details 内部函数，用于状态机切换主运行状态
 */
void motor_tran_state(struct motor *motor, enum motor_status new_state)
{

	struct statemachine *sm = &motor->sm;
	if (!motor || !sm) {
		return;
	}

	switch (new_state) {
	case MOTOR_STATUS_INIT:
		sm_transition(sm, motor_init_state);
		break;

	case MOTOR_STATUS_CALIB:
		sm_transition(sm, motor_calib_state);
		break;

	case MOTOR_STATUS_IDLE:
		sm_transition(sm, motor_idle_state);
		break;

	case MOTOR_STATUS_RUNNING:
		sm_transition(sm, motor_runing_state);
		break;

	default:
		break;
	}
}
