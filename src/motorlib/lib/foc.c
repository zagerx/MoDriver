#include "_motorlib_internal.h"
#include "arm_math_types.h"
#include "foc.h"

#include "foc_pid.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "svpwm.h"
#include "arm_math.h"
#include "motor_driver.h"
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
	foc->ctrl.d_axis.params = d_axis_pid_param;
	foc->ctrl.q_axis.params = q_axis_pid_param;
	foc->ctrl.velocity.params = vel_pid_param;
	foc->ctrl.position.params = pos_pid_param;
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
	struct foc_measurement *meas = &foc->meas;
	struct currsmp_output out;
	currsmp_get_output(foc->currsmp, &out);
	float i_a = out.i_a;
	float i_b = out.i_b;

	float eangle = feedback_get_elec_angle(foc->feedback);

	arm_clarke_f32(i_a, i_b, &meas->i_alpha, &meas->i_beta);

	float sin_val, cos_val;

	arm_sin_cos_f32(eangle * RAD_TO_DEG, &sin_val, &cos_val);
	arm_park_f32(meas->i_alpha, meas->i_beta, &meas->i_d, &meas->i_q, sin_val, cos_val);
}
