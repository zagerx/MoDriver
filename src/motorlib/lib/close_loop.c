/* SPDX-License-Identifier: GPL-2.0 */

/**
 * @file close_loop.c
 * @brief 电机闭环控制实现
 *
 * 本文件包含电机速度闭环控制的相关函数实现。
 */

#include "close_loop.h"
#include "currsmp.h"
#include "feedback.h"
#include "foc.h"
#include "inverter.h"
#include "_motorlib_internal.h"
#include "foc_pid.h"
#include "svpwm.h"
#include "motorlib_control_param.h"
/**
 * @brief 电机速度闭环控制
 *
 * 执行速度环PI控制，根据目标速度和实际速度计算q轴电流参考值，
 * 并将d轴电流参考值设为0。
 *
 * @param[in,out] motor 电机对象指针，包含FOC参数、反馈信息和控制器状态
 *
 * @return 无返回值
 */
void motor_velocity_loop(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_param *param = foc->parm;
	struct foc_measurement *meas = &foc->meas;
	float target = param->target_vel;
	float vel = meas->fd_out->velocity_rad_s;

	foc->ref.i_q = foc_pid_run(velocity_pi, target, vel, SPEED_PERIOD_DT);
	foc->ref.i_d = 0.0f;
}
void motor_currment_loop(struct motor *motor)
{

	struct foc *foc = &motor->foc;

	struct foc_measurement *meas = &foc->meas;
	struct foc_control *ctrl = &foc->ctrl;
	struct foc_pid *d_axis_pid = &ctrl->d_axis;
	struct foc_pid *q_axis_pid = &ctrl->q_axis;
	float ud, uq;
	float ualpha, ubeta;
	float duty[3];
	ud = foc_currentloop_pid_run(d_axis_pid, foc->ref.i_d, meas->i_d, CONTROL_PERIOD_DT);
	uq = foc_currentloop_pid_run(q_axis_pid, foc->ref.i_q, meas->i_q, CONTROL_PERIOD_DT);
	float ud_limit = ud;
	float uq_limit = uq;

	float vbus = meas->cs_out->v_bus;
	float eangle = meas->fd_out->eangle_rad;
	svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
	foc_currentpid_saturation(&foc->ctrl.d_axis, ud_limit, ud);
	foc_currentpid_saturation(&foc->ctrl.q_axis, uq_limit, uq);
	svpwm_normalize(eangle, vbus, ud_limit, uq_limit, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
