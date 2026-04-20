/**
 * @file foc.c
 * @brief FOC（磁场定向控制）模块实现
 * @details 实现FOC数据源绑定及Clarke/Park变换，更新d/q轴电流测量值
 */

#include "_motorlib_internal.h"
// #include "arm_math_types.h"
#include "foc.h"

#include "foc_pid.h"
#include "feedback.h"
#include "currsmp.h"
#include "arm_math.h"
#include "motorlib_constants.h"

/**
 * @brief 绑定FOC数据源
 * @param[in] foc FOC实例
 * @param[in] feedback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @param[in] foc_param FOC参数
 * @return 无
 */
void foc_bind(struct foc *foc, struct feedback *feedback, struct currsmp *currsmp,
	      struct foc_param *foc_param)
{
	if (!foc) {
		return;
	}

	foc->meas.cs_out = &currsmp->output;
	foc->meas.fd_out = &feedback->output;
	foc->ctrl.d_axis.params = &foc_param->d_axis;
	foc->ctrl.q_axis.params = &foc_param->q_axis;
	foc->ctrl.velocity.params = &foc_param->vel;
	foc->ctrl.position.params = &foc_param->pos;
	foc->parm = foc_param;
}

/**
 * @brief 更新id/iq电流
 * @param[in,out] foc FOC实例
 * @return 无
 * @details 使用Clarke和Park变换计算d/q轴电流
 */
void foc_update_idiq(struct foc *foc)
{
	if (!foc) {
		return;
	}

	struct foc_measurement *meas = &foc->meas;

	float i_a = meas->cs_out->i_a;
	float i_b = meas->cs_out->i_b;

	float eangle = meas->fd_out->eangle_rad;
	arm_clarke_f32(i_a, i_b, &meas->i_alpha, &meas->i_beta);

	float sin_val, cos_val;

	arm_sin_cos_f32(eangle * MOTORLIB_RAD_TO_DEG, &sin_val, &cos_val);
	arm_park_f32(meas->i_alpha, meas->i_beta, &meas->i_d, &meas->i_q, sin_val, cos_val);
}
