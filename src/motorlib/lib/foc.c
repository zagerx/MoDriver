#include "_motorlib_internal.h"
#include "arm_math_types.h"
#include "foc.h"

#include "foc_data.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "svpwm.h"
#include "arm_math.h"
#undef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
void foc_bind(struct foc *foc, struct feedback_output *fb_out, struct currsmp_output *currsmp_out)
{
	if (foc) {
		foc_data_bind(&foc->data, fb_out, currsmp_out);
	}
}
void foc_update_idiq(struct foc *foc)
{
	if (!foc) {
		return;
	}
	struct foc_data *data = &foc->data;
	struct foc_measurement *meas = &data->meas;

	float i_a = meas->currsmp->i_a;
	float i_b = meas->currsmp->i_b;

	float eangle = meas->fb_out->eangle_rad;

	arm_clarke_f32(i_a, i_b, &meas->i_alpha, &meas->i_beta);

	float sin_val, cos_val;

	arm_sin_cos_f32(eangle * RAD_TO_DEG, &sin_val, &cos_val);
	arm_park_f32(meas->i_alpha, meas->i_beta, &meas->i_d, &meas->i_q, sin_val, cos_val);
}
void open_loop_force_align(struct motor *motor, float d_axis_voltage, float eangle)
{
	if (!motor) {
		return;
	}
	float uq = 0.0f; // q轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];
	float vbus = motor->currsmp->output.v_bus;
	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(eangle, vbus, d_axis_voltage, 0.0f, &ualpha, &ubeta); // 归一化到线性调制区
	// 直接输出d轴电压，q轴为0，保持固定角度eangle
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}

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
	float vbus = motor->currsmp->output.v_bus;
	svpwm_limit_voltage(vbus, &d_axis_voltage, &uq);
	svpwm_normalize(foc_data->self_eangle, vbus, d_axis_voltage, 0.0f, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}

float open_loop_get_force_angle(struct motor *motor)
{
	if (!motor) {
		return 0.0f;
	}
	return motor->foc.data.self_eangle;
}

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

	struct foc_data *foc_data = &motor->foc.data;
	struct foc_measurement *meas = &foc_data->meas;
	float vbus = meas->currsmp->v_bus;
	svpwm_limit_voltage(vbus, &ud, &q_axis_voltage);
	svpwm_normalize(eangle, vbus, ud, q_axis_voltage, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
