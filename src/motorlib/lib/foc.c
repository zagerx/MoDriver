#include "_motorlib_internal.h"
#include "arm_math_types.h"
#include "foc.h"

#include "foc_data.h"
#include "foc_pid.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "svpwm.h"
#include "arm_math.h"
#include "motorlib_control_param.h"
#undef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
/**
 * @brief 绑定FOC数据源
 * @param[in] foc FOC实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_bind(struct foc *foc, struct feedback *feeback, struct currsmp *currsmp,
	      struct foc_pid_param *d_axis_pid_param, struct foc_pid_param *q_axis_pid_param,
	      struct foc_pid_param *vel_pid_param, struct foc_pid_param *pos_pid_param)
{
	if (!foc) {
		return;
	}
	foc->feedback = feeback;
	foc->currsmp = currsmp;
	foc_data_bind(&foc->data, d_axis_pid_param, q_axis_pid_param, vel_pid_param, pos_pid_param);
}
/**
 * @brief 更新id/iq电流
 * @param[in] foc FOC实例
 * @return 无
 * @details 使用Clarke和Park变换计算d/q轴电流
 */
void foc_update_idiq(struct foc *foc)
{
	if (!foc || !foc->currsmp || !foc->feedback) {
		return;
	}
	struct foc_data *data = &foc->data;
	struct foc_measurement *meas = &data->meas;
	struct currsmp_output out;
	currsmp_get_output(foc->currsmp, &out);
	// float vbus = out.v_bus;
	float i_a = out.i_a;
	float i_b = out.i_b;

	float eangle = feedback_get_elec_angle(foc->feedback);

	arm_clarke_f32(i_a, i_b, &meas->i_alpha, &meas->i_beta);

	float sin_val, cos_val;

	arm_sin_cos_f32(eangle * RAD_TO_DEG, &sin_val, &cos_val);
	arm_park_f32(meas->i_alpha, meas->i_beta, &meas->i_d, &meas->i_q, sin_val, cos_val);
}
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
	if (!motor) {
		return;
	}
	float uq = 0.0f; // q轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];
	struct currsmp_output out;
	currsmp_get_output(motor->currsmp, &out);
	float vbus; // = motor->currsmp->output.v_bus;
	vbus = out.v_bus;
	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(eangle, vbus, d_axis_voltage, uq, &ualpha, &ubeta); // 归一化到线性调制区
	// 直接输出d轴电压，q轴为0，保持固定角度eangle
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
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
	if (!motor) {
		return;
	}
	struct foc_data *foc_data = &motor->foc.data;
	foc_data->self_eangle += omega * dt; // 电角度增量 = 角速度 * 时间
	float uq = 0.0f;                     // q轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];
	// float vbus = motor->currsmp->output.v_bus;
	struct currsmp_output out;
	currsmp_get_output(motor->currsmp, &out);
	float vbus = out.v_bus;
	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(foc_data->self_eangle, vbus, d_axis_voltage, 0.0f, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}

/**
 * @brief 获取开环强制角度
 * @param[in] motor 电机实例
 * @return 当前电角度
 */
float open_loop_get_force_angle(struct motor *motor)
{
	if (!motor) {
		return 0.0f;
	}
	return motor->foc.data.self_eangle;
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
	if (!motor) {
		return;
	}
	struct feedback *feedback = motor->feedback;
	float eangle = feedback_get_elec_angle(feedback);
	float ud = 0.0f; // d轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];

	// struct foc_data *foc_data = &motor->foc.data;
	// struct foc_measurement *meas = &foc_data->meas;
	// float vbus = meas->currsmp->v_bus;
	struct currsmp_output out;
	currsmp_get_output(motor->currsmp, &out);
	float vbus = out.v_bus;
	svpwm_limit_voltage(vbus, &ud, &q_axis_voltage);
	svpwm_normalize(eangle, vbus, ud, q_axis_voltage, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
void currment_debug(struct motor *motor, float tar)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = motor->feedback;
	float eangle = feedback_get_elec_angle(feedback);
	float ud, uq; // d轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];

	struct foc_data *foc_data = &motor->foc.data;
	struct currsmp_output out;
	currsmp_get_output(motor->currsmp, &out);
	float vbus = out.v_bus;
	struct foc_measurement *meas = &foc_data->meas;
	ud = foc_currentloop_pid_run(&foc_data->ctrl.d_axis, tar, meas->i_d, CONTROL_PERIOD_DT);
	uq = 0.0f;
	float ud_limit = ud;
	float uq_limit = uq;
	svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
	foc_currentpid_saturation(&foc_data->ctrl.d_axis, ud_limit, ud);
	svpwm_normalize(eangle, vbus, ud_limit, uq, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
