/**
 * @file open_loop.c
 * @brief 电机开环控制实现
 * @details 实现开环强制对齐、强制拖动、编码器反馈开环运行及电流调试模式
 */

#include "open_loop.h"
#include "_motorlib_internal.h"
#include "svpwm.h"
#include "inverter.h"
#include "currsmp.h"
#include "feedback.h"
#include "motorlib_control_param.h"

/**
 * @brief 开环强制对齐（固定角度）
 * @param[in] motor 电机实例
 * @param[in] d_axis_voltage d轴电压
 * @param[in] eangle 电角度
 * @return 无
 * @details 输出固定电角度的d轴电压，用于电机初始对齐
 */
void open_loop_force_align(struct motor *motor, float d_axis_voltage, float eangle)
{
	if (!motor)
	{
		return;
	}
	struct currsmp *currsmp = &motor->currsmp;
	struct inverter *inverter = &motor->inverter;
	float uq = 0.0f;
	float ualpha, ubeta;
	float duty[3];
	struct currsmp_output out;

	currsmp_get_output(currsmp, &out);

	float vbus;
	vbus = out.v_bus;

	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(eangle, vbus, d_axis_voltage, uq, &ualpha, &ubeta);
	// 直接输出d轴电压，q轴为0，保持固定角度eangle
	svpwm_calc_duty(ualpha, ubeta, duty);

	inverter_set_voltage(inverter, duty[0], duty[1], duty[2]);
}

/**
 * @brief 开环强制拖动（角速度控制）
 * @param[in] motor 电机实例
 * @param[in] dt 时间间隔
 * @param[in] d_axis_voltage d轴电压
 * @param[in] omega 角速度
 * @return 无
 * @details 根据角速度积分更新电角度，输出对应电压
 */
void open_loop_force_drag(struct motor *motor, float dt, float d_axis_voltage, float omega)
{
	if (!motor)
	{
		return;
	}
	struct currsmp *currsmp = &motor->currsmp;
	struct inverter *inverter = &motor->inverter;
	struct foc *foc = &motor->foc;
	foc->self_eangle += omega * dt; // 电角度增量 = 角速度 * 时间

	float uq = 0.0f;
	float ualpha, ubeta;
	float duty[3];
	struct currsmp_output out;

	currsmp_get_output(currsmp, &out);

	float vbus = out.v_bus;

	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(foc->self_eangle, vbus, d_axis_voltage, 0.0f, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);

	inverter_set_voltage(inverter, duty[0], duty[1], duty[2]);
}

/**
 * @brief 获取开环强制角度
 * @param[in] motor 电机实例
 * @return 当前电角度
 */
float open_loop_get_force_angle(struct motor *motor)
{
	if (!motor)
	{
		return 0.0f;
	}

	return motor->foc.self_eangle;
}

/**
 * @brief 开环编码器模式（使用编码器反馈）
 * @param[in] motor 电机实例
 * @param[in] q_axis_voltage q轴电压
 * @return 无
 * @details 使用编码器反馈的电角度，输出q轴电压
 */
void open_loop_encoder(struct motor *motor, float q_axis_voltage)
{
	if (!motor)
	{
		return;
	}
	struct currsmp *currsmp = &motor->currsmp;
	struct inverter *inverter = &motor->inverter;
	struct feedback *feedback = &motor->feedback;
	float eangle = feedback_get_elec_angle(feedback);
	float ud = 0.0f;
	float ualpha, ubeta;
	float duty[3];

	struct currsmp_output out;
	currsmp_get_output(currsmp, &out);

	float vbus = out.v_bus;

	svpwm_limit_voltage(vbus, &ud, &q_axis_voltage);
	svpwm_normalize(eangle, vbus, ud, q_axis_voltage, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);

	inverter_set_voltage(inverter, duty[0], duty[1], duty[2]);
}

/**
 * @brief d轴电流调试模式
 * @param[in] motor 电机实例
 * @param[in] tar 目标电流值
 * @return 无
 * @details 使用FOC电流环PID控制，输出d轴电压
 */
void currment_debug(struct motor *motor, float tar)
{
	if (!motor)
	{
		return;
	}
	struct inverter *inverter = &motor->inverter;
	struct foc *foc = &motor->foc;
	struct foc_measurement *meas = &foc->meas;

	float eangle = meas->fd_out->eangle_rad;
	float ud, uq;
	float ualpha, ubeta;
	float duty[3];

	float vbus = meas->cs_out->v_bus; // out.v_bus;

	ud = foc_currentloop_pid_run(&foc->ctrl.d_axis, tar, meas->i_d, CONTROL_PERIOD_DT);
	uq = 0.0f;

	float ud_limit = ud;
	float uq_limit = uq;

	svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
	foc_currentpid_saturation(&foc->ctrl.d_axis, ud_limit, ud);

	svpwm_normalize(eangle, vbus, ud_limit, uq_limit, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);

	inverter_set_voltage(inverter, duty[0], duty[1], duty[2]);
}
