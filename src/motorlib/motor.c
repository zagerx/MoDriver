
/**
 * @file motor.c
 * @brief 电机控制模块主文件
 * @author zager
 * @date 2026-03-18
 * @version v0.0.9
 *
 * 本文件实现电机控制的核心功能，包括硬件绑定、参数绑定、初始化和高频控制任务。
 * 模块采用状态机设计，支持校准、空闲、运行等多种工作状态。
 */

#include "motor.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "statemachine.h"
#include "motor_state.h"
#include "foc.h"
#include "motorlib_control_param.h"
#include "motor_mode.h"
#include <stdint.h>

#undef NULL
#define NULL (0)

/**
 * @brief 绑定硬件接口
 * @param[in] motor 电机实例指针
 * @param[in] hw 硬件接口集合指针
 * @return 无
 * @details 将编码器操作接口和逆变器操作接口绑定到电机实例。
 *          必须在初始化前调用此函数。
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
 * @param[in] motor 电机实例指针
 * @param[in] param_ext 扩展参数指针
 * @return 无
 * @details 绑定反馈参数、电流采样参数和FOC参数到电机实例。
 *          必须在初始化前调用此函数。
 */
void motor_bind_param_ext(struct motor *motor, struct motor_param_ext *param_ext)
{
	if (!motor || !param_ext) {
		return;
	}

	motor->param_ext = param_ext;
	feedback_bind_encoder_param(motor->feedback, &param_ext->feedback_param);
	currsmp_bind_param(motor->currsmp, &param_ext->currsmp_param);
	trajectory_planner_bind_param(&motor->traj_plan, &param_ext->traj_param);
}

/**
 * @brief 检查电机参数合法性
 * @param[in] motor 电机实例指针
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
 * @note 当前版本总是返回-1，强制进入校准状态
 */
static int16_t motor_param_check(struct motor *motor)
{
	if (!motor) {
		return -1; /* 电机实例为空 */
	}

	struct motor_param_ext *param_ext = motor->param_ext;
	if (!param_ext) {
		return -2; /* 扩展参数为空 */
	}

	/* CRC 校验伪代码 */
	/* uint16_t calc_crc = crc16_calculate((uint8_t *)param_ext, sizeof(*param_ext)); */
	/* if (calc_crc != param_ext->crc_16) { */
	/*     return -20; */
	/* } */
	(void)param_ext->crc_16;

	/* 暂时保留默认返回-1，确保进入校准状态 */
	return -1;
}

/**
 * @brief 初始化电机
 * @param[in] motor 电机实例指针
 * @return 无
 * @details 检查参数合法性并初始化状态机。
 *          如果参数检查失败，进入校准状态；否则进入空闲状态。
 *          同时绑定FOC控制器参数。
 */
void motor_init(struct motor *motor)
{
	if (!motor) {
		return;
	}

	struct statemachine *sm = &motor->sm;
	struct statemachine *sm_mode = &motor->sm_mode;
	struct feedback *fb = motor->feedback;
	struct currsmp *currsmp = motor->currsmp;

	if (!sm || !fb || !currsmp) {
		/* 关键指针为空，无法初始化 */
		return;
	}

	if (motor_param_check(motor)) {
		statemachine_init(sm, motor, motor_carib_state, NULL, 0);
	} else {
		statemachine_init(sm, motor, motor_idle_state, NULL, 0);
	}

	statemachine_init(sm_mode, motor, motor_mode_none, NULL, 0);
	struct foc *foc = &motor->foc;
	if (!motor->param_ext) {
		/* 参数未绑定 */
		return;
	}

	foc_bind(foc, fb, currsmp, &motor->param_ext->foc_param);
}

/**
 * @brief 高频控制任务
 * @param[in] motor 电机实例指针
 * @param[in] adc_raw ADC原始数据数组指针
 * @return 无
 * @note 应在定时器中断中周期性调用（默认10kHz）
 * @details 执行电流采样原始数据更新、反馈原始数据更新和状态机调度。
 *          在非校准状态下，执行完整的反馈更新和FOC电流计算。
 *          在校准状态下，仅更新母线相关数据。
 */
void motor_highfreq_task(struct motor *motor, uint16_t *adc_raw)
{
	if (!motor) {
		return;
	}

	struct feedback *feedback = motor->feedback;
	struct currsmp *currsmp = motor->currsmp;
	struct statemachine *sm = &motor->sm;

	currsmp_update_raw(currsmp, adc_raw);
	feedback_update_raw(feedback);

	/* 仅在非校准状态下执行完整的反馈更新和状态机调度，校准状态下可能需要特殊处理 */
	if (sm->current_state != motor_carib_state) {
		currsmp_update(currsmp);
		feedback_update(feedback, CONTROL_PERIOD_DT);
		foc_update_idiq(&motor->foc);
	} else {
		currsmp_update_bus(currsmp);
	}

	sm_dispatch(sm);
}

void motor_tran_runing(struct motor *motor)
{
	if (!motor) {
		return;
	}

	_tran_state(motor, MOTOR_STATUS_RUNING);
}
void motor_tran_idle(struct motor *motor)
{
	if (!motor) {
		return;
	}

	_tran_state(motor, MOTOR_STATUS_IDLE);
}

void motor_set_target_pos(struct motor *motor, float target_pos, float target_vel)
{
	if (!motor) {
		return;
	}
	struct trajectory_plan *traj_plan = &motor->traj_plan;
	trajectory_planner_update_target(traj_plan, target_pos, target_vel);
}