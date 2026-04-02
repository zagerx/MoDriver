/**
 * @file motor_mode.c
 * @brief 电机操作模式状态机实现
 * @details 实现CiA 402轮廓位置(PP)、轮廓速度(PV)、原点回归(HM)等模式的状态处理
 */

#include "foc_pid.h"
#include "motor_mode.h"
#include "statemachine.h"
#include "close_loop.h"
#include "motorlib_control_param.h"
#include "_motorlib_internal.h"
#include <stdint.h>
/**
 * @brief 电机无模式状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 当电机未分配任何操作模式时调用此状态函数
 */
void motor_mode_none(struct statemachine *sm)
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
 * @brief 电机轮廓位置模式状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 执行轮廓位置模式（PP）控制，内部规划轨迹并调用位置环/速度环/电流环
 */
void motor_mode_PP(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	struct trajectory_plan *traj_plan = &motor->traj_plan;
	struct foc *foc = &motor->foc;
	float start_pos = foc->meas.fd_out->odometer; /* 当前位置作为起始位置 */
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		sm->count = 0;
		trajectory_planner_init(traj_plan, start_pos, 0.0f, 0.0f, POSITION_PERIOD_DT);
		motor_position_loop_reset(motor);
#if MOTORLIB_DEBUG_ENABLED
		motor->data.debug.test_flag1 = 1;
		motor->data.debug.test_flag2 = 1;
#endif
		break;

	case RUNING:
		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			trajectory_planner_action(traj_plan, POSITION_PERIOD_DT);
			motor_position_loop(motor, POSITION_PERIOD_DT);
#if MOTORLIB_DEBUG_ENABLED
			motor->data.debug.test_flag1 = -motor->data.debug.test_flag1;
#endif
		}
		if (sm->count % (uint16_t)(SPEED_LOOP_INTERVAL) == 0) {
			motor_velocity_loop(motor, foc->ref.velocity);
#if MOTORLIB_DEBUG_ENABLED
			motor->data.debug.test_flag2 = -motor->data.debug.test_flag2;
#endif
		}
		motor_currment_loop(motor);
		sm->count++;
		break;

	case EXIT:
		break;

	default:
		break;
	}
}
/**
 * @brief 电机轮廓速度模式状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 执行轮廓速度模式（PV）控制，内部规划轨迹并调用速度环/电流环
 */
void motor_mode_PV(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	struct trajectory_plan *traj_plan = &motor->traj_plan;

	switch (sm->phase) {
	case ENTER:
		sm->count = 0;
		sm->phase = RUNING;
		break;

	case RUNING:
		if (sm->count++ > SPEED_LOOP_INTERVAL) {
			sm->count = 0;
			trajectory_planner_action(traj_plan, SPEED_PERIOD_DT);
			float target_vel = trajectory_planner_get_vel(&motor->traj_plan);
			motor_velocity_loop(motor, target_vel);
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
 * @brief 电机原点回归模式状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 执行原点回归（Homing）操作，寻找机械零点
 */
void motor_mode_HOMING(struct statemachine *sm)
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
 * @brief 获取电机当前操作模式
 * @param[in] motor 电机实例指针
 * @return enum motor_mode 当前操作模式
 * @details 通过模式状态机 current_state 反查枚举值
 */
enum motor_mode motor_get_mode(const struct motor *motor)
{
	if (!motor) {
		return MODE_NONE;
	}

	sm_state_t state = motor->sm_mode.current_state;
	if (state == motor_mode_PP) {
		return MODE_PP;
	}
	if (state == motor_mode_PV) {
		return MODE_PV;
	}
	if (state == motor_mode_HOMING) {
		return MODE_HM;
	}
	return MODE_NONE;
}

/**
 * @brief 切换电机操作模式
 * @param[in] motor 电机实例指针
 * @param[in] new_mode 新的操作模式
 * @details 内部函数，用于状态机切换操作模式
 */
void motor_tran_mode(struct motor *motor, enum motor_mode new_mode)
{
	struct statemachine *sm_mode = &motor->sm_mode;
	switch (new_mode) {
	case MODE_NONE:
		if (sm_mode->current_state != motor_mode_none) {
			TRAN_STATE(sm_mode, motor_mode_none);
		}
		break;

	case MODE_PP:
		if (sm_mode->current_state != motor_mode_PP) {
			TRAN_STATE(sm_mode, motor_mode_PP);
		}

		break;

	case MODE_PV:
		if (sm_mode->current_state != motor_mode_PV) {
			TRAN_STATE(sm_mode, motor_mode_PV);
		}
		break;

	case MODE_HM:
		if (sm_mode->current_state != motor_mode_HOMING) {
			TRAN_STATE(sm_mode, motor_mode_HOMING);
		}
		break;

	default:
		break;
	}
}