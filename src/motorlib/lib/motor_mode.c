/**
 * @file motor_mode.c
 * @brief 电机操作模式状态机实现
 * @details 实现CiA 402轮廓位置(PP)、轮廓速度(PV)、原点回归(HM)等模式的状态处理
 */

#include "foc_pid.h"
#include "motor_interface_mode.h"
#include "motor_interface_params.h"
#include "motor_mode.h"
#include "statemachine.h"
#include "close_loop.h"
#include "motorlib_control_param.h"
#include "_motorlib_internal.h"
#include <stdint.h>

#include <math.h>
#include "open_loop.h"
extern int motor_is_command_set(const struct motor *motor, enum motor_command_bits bit);
extern int motor_clear_command(struct motor *motor, enum motor_command_bits bit);

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
	float start_pos; // foc->meas.fd_out->odometer; /* 当前位置作为起始位置 */
	float plan_target, actual_pos;
	struct motor_param_ext *param_ext = motor->param_ext;
	float wheel_radius = param_ext->electrical_param.wheel_radius; /* 轮子半径，单位mm */
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		sm->count = 0;
		start_pos = foc->meas.fd_out->mangle_rad * wheel_radius; /* 当前位置作为起始位置 */
		trajectory_planner_init(traj_plan, start_pos, 0.0f, 0.0f, POSITION_PERIOD_DT);
		motor_position_loop_reset(motor);
		break;

	case RUNING:
#define TARGET_REACHED_POS_TOL 0.1f // 位置容差：0.1 mm
#define TARGET_REACHED_VEL_TOL 5.0f // 速度容差：1 mm/s

		/* 判断目标到达 */
		plan_target = trajectory_planner_read_plantarget(traj_plan); // 目标位置
		actual_pos = foc->meas.fd_out->mangle_rad * wheel_radius;    // 实际位置

		if (fabsf(actual_pos - plan_target) < TARGET_REACHED_POS_TOL) {
			motor_set_flag(motor, MOTOR_FLAGS_TARGET_REACHED);
		} else {
			motor_clear_flag(motor, MOTOR_FLAGS_TARGET_REACHED);
		}

		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			trajectory_planner_action(traj_plan, POSITION_PERIOD_DT);
			float plan_position = trajectory_planner_get_pos(traj_plan) / wheel_radius;
			foc->ref.velocity =
				motor_position_loop(motor, plan_position, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)(SPEED_LOOP_INTERVAL) == 0) {
			float plan_velocity = trajectory_planner_get_vel(traj_plan) / wheel_radius;
			motor_velocity_loop(motor, foc->ref.velocity + plan_velocity);
		}
		motor_currment_loop(motor);
		sm->count++;
		break;

	case EXIT:
		motor_position_loop_reset(motor);
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
		motor_velocity_loop_reset(motor);
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
		WAIT_COMMAND,
		IDLE,
	};
#define CAPTURE_THRESHOLD 0.001f // 约0.57度

	struct motor *motor = (struct motor *)(sm->data);
	struct foc *foc = &motor->foc;
	float elec_angle;
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		motor_velocity_loop_reset(motor);
		motor_clear_command(motor, MOTOR_CMD_HOMING);
		sm->count = 0;
		sm->phase = WAIT_COMMAND;
		break;

	case WAIT_COMMAND:

		/* 等待外部命令触发原点回归动作 */
		if (motor_is_command_set(motor, MOTOR_CMD_HOMING)) {
			motor_clear_command(motor, MOTOR_CMD_HOMING);
			sm->phase = RUNING;
		}

		break;

	case RUNING:
		elec_angle = foc->meas.fd_out->eangle_rad;
		if (fabsf(elec_angle) < CAPTURE_THRESHOLD ||
		    fabsf(elec_angle - (3.141592653f * 2.0f)) < CAPTURE_THRESHOLD) {
			motor_set_flag(motor, MOTOR_FLAGS_HOMING_DONE);
			sm->count = 0;
			motor_velocity_loop_reset(motor);
			inverter_set_voltage(&motor->inverter, 0.0f, 0.0f, 0.0f); // 锁定位置
			feedback_reset_odometer(&motor->feedback); // 将当前位置设为零点
			sm->phase = WAIT_COMMAND;
			break;
		}

		if (sm->count++ > SPEED_LOOP_INTERVAL) {
			sm->count = 0;
			motor_velocity_loop(motor, 5.0f);
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
 * @brief 电机d轴电流调试模式
 * @param[in] sm 状态机实例
 * @details 通过d轴电流环PID控制进行电流调试
 */
#include "motorlib_control_param.h"
void motor_mode_debug(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
		ALIGN,
		RUNING2,
		IDLE,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = &motor->inverter;
	static float target = 0.0f;
	static uint32_t test_flag = 1;
	switch (sm->phase) {
	case ENTER:
		inverter_enable(inverter);
		sm->count = 0;
		sm->phase = RUNING;
		break;

	case ALIGN:
		open_loop_encoder(motor, ALIGN_VOLTAGE);
		if (++sm->count > 30000) // 1秒后切换到运行状态
		{
			sm->count = 0;
			sm->phase = IDLE;
		}
		break;

	case RUNING:

		if (++sm->count > 500) {
			test_flag = -test_flag; // 反转目标位置
			sm->count = 0;

			if (test_flag == 1) {
				target = 4.0f; // 正转目标位置
			} else {
				target = 0.0f; // 反转目标位置
			}
			// 反转目标位置
			foc_pid_reset(&motor->foc.ctrl.d_axis); // 重置PID控制器状态
			break;
		}
		currment_debug(motor, target);
		break;

	case EXIT:
		inverter_disable(inverter);
		break;

	default:
		break;
	}
}
void motor_mode_debug_posvel(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct foc *foc = &motor->foc;
	switch (sm->phase) {
	case ENTER:
		sm->count = 0;
		motor_position_loop_reset(motor);
		motor->data.debug.test_value1 = foc->meas.fd_out->mangle_rad;
		sm->phase = RUNING;
		break;

	case RUNING:
		/* 执行控制环 */
		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			/* 位置环 */
			float target_linear = motor->data.debug.test_value1;
			foc->ref.velocity =
				motor_position_loop(motor, target_linear, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)SPEED_LOOP_INTERVAL == 0) {
			motor_velocity_loop(motor, foc->ref.velocity);
		}
		/* 电流环 */
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
	if (state == motor_mode_debug) {
		return MODE_DEBUG;
	}
	if (state == motor_mode_anticogging_calib) {
		return MODE_ANTICOGGING_CALIB;
	}
	if (state == motor_mode_debug_posvel) {
		return MODE_DEBUG_POSVEL;
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
	case MODE_DEBUG: {
		if (sm_mode->current_state != motor_mode_debug) {
			TRAN_STATE(sm_mode, motor_mode_debug);
		}
	} break;
	case MODE_ANTICOGGING_CALIB: {
		if (sm_mode->current_state != motor_mode_anticogging_calib) {
			TRAN_STATE(sm_mode, motor_mode_anticogging_calib);
		}
	} break;
	case MODE_DEBUG_POSVEL: {
		if (sm_mode->current_state != motor_mode_debug_posvel) {
			TRAN_STATE(sm_mode, motor_mode_debug_posvel);
		}
	} break;
	default:
		break;
	}
}
