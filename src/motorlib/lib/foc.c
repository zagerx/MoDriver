#include "_motorlib_internal.h"
#include "foc.h"

#include "foc_data.h"
#include "inverter.h"
#include "svpwm.h"
void foc_bind(struct foc *foc, struct feedback_output *fb_out)
{
	if (foc) {
		foc_data_bind(&foc->data, fb_out);
	}
}

void open_loop_force_align(struct motor *motor, float d_axis_voltage, float eangle)
{
	if (!motor) {
		return;
	}
	float uq = 0.0f; // q轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];
	svpwm_limit_voltage(24.0f, &d_axis_voltage, &uq); // 限幅到12V，q轴电压为0
	svpwm_normalize(eangle, 24.0f, d_axis_voltage, 0.0f, &ualpha, &ubeta); // 归一化到线性调制区
	// 直接输出d轴电压，q轴为0，保持固定角度eangle
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}

/*周期性调用 */
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
	svpwm_limit_voltage(24.0f, &d_axis_voltage, &uq);
	svpwm_normalize(foc_data->self_eangle, 24.0f, d_axis_voltage, 0.0f, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
