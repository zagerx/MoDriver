#include "feedback.h"
#include <stdint.h>
#include <math.h>

/* 低通滤波系数（0~1，越大响应越快） */
#define VELOCITY_LPF_ALPHA 0.08f

/* 角度归一化到 [0, 2PI] */
static float normalize_angle(float angle)
{
	angle = fmodf(angle, 2.0f * M_PI);
	if (angle < 0.0f) {
		angle += 2.0f * M_PI;
	}
	return angle;
}

/* 绑定编码器操作接口 */
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops)
{
	if (feedback) {
		feedback->ops = ops;
	}
}
void feedback_bind_encoder_param(struct feedback *feedback, struct feedback_param *param)
{
	if (feedback) {
		feedback->param = param;
	}
}
/* 初始化反馈模块，校验参数合法性 */
enum feedback_error_code feedback_init(struct feedback *feedback)
{
	if (!feedback) {
		return FEEDBACK_ERROR_PARAM;
	}

	if (!feedback->ops || !feedback->ops->read) {
		return FEEDBACK_ERROR_HW_FAILURE;
	}

	struct feedback_param *param = feedback->param;
	if (param->wheel_radius <= 0.0f || param->gear_ratio <= 0.0f || param->pole_pairs <= 0.0f ||
	    (param->direction != 1.0f && param->direction != -1.0f) ||
	    param->encoder_resolution == 0) {
		return FEEDBACK_ERROR_PARAM;
	}

	feedback->state = FEEDBACK_STATE_OK;
	struct feedback_data *data = &feedback->data;
	data->raw = 0;
	data->prev_raw = 0;
	data->total_counts = 0;
	data->accumulated_mangle_rad = 0.0f;
	data->prev_mangle_rad = 0.0f;
	data->mech_omega_rad_s = 0.0f;

	feedback->output.eangle_rad = 0.0f;
	feedback->output.velocity_rad_s = 0.0f;
	feedback->output.odometer = 0.0f;

	return FEEDBACK_ERROR_NONE;
}

/* 计算累积机械角度，处理编码器溢出 */
static void feedback_calc_accumulated_mangle(struct feedback *feedback, int32_t adjusted_raw)
{
	struct feedback_param *param = feedback->param;
	struct feedback_data *data = &feedback->data;

	const float two_pi = 2.0f * M_PI;
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 计算原始差值（带溢出处理） */
	int32_t delta = (int32_t)adjusted_raw - (int32_t)data->prev_raw;

	/* 处理越过CPR边界的溢出 */
	if (delta > cpr / 2) {
		delta -= cpr;
	} else if (delta < -cpr / 2) {
		delta += cpr;
	}

	/* 更新累积计数和机械角度 */
	data->total_counts += delta;
	data->accumulated_mangle_rad =
		(two_pi / (float)cpr) * (float)data->total_counts * param->direction;

	/* 保存当前值供下次使用 */
	data->prev_raw = (uint16_t)adjusted_raw;
}

/* 计算电角度（机械角度 * 极对数，归一化到 [0, 2π]） */
static float feedback_calc_elec_angle(struct feedback *feedback, float mangle)
{
	struct feedback_param *param = feedback->param;
	return normalize_angle(mangle * param->pole_pairs);
}

/* 差分法计算机械角速度，带低通滤波 */
static float feedback_calc_velocity(struct feedback *feedback, float dt, float cur_mangle)
{
	struct feedback_data *data = &feedback->data;

	if (dt <= 0.0f) {
		return data->mech_omega_rad_s;
	}

	/* 计算角度差（处理跨越 2π 边界） */
	float dtheta = cur_mangle - data->prev_mangle_rad;
	if (dtheta > M_PI) {
		dtheta -= 2.0f * M_PI;
	} else if (dtheta < -M_PI) {
		dtheta += 2.0f * M_PI;
	}

	/* 原始速度计算 */
	float speed_raw = dtheta / dt;

	/* 一阶低通滤波 */
	data->mech_omega_rad_s = (1.0f - VELOCITY_LPF_ALPHA) * data->mech_omega_rad_s +
				 VELOCITY_LPF_ALPHA * speed_raw;

	/* 保存当前角度供下次差分使用 */
	data->prev_mangle_rad = cur_mangle;

	return data->mech_omega_rad_s;
}

/* 更新反馈数据（编码器读取 + 角度/速度/电角度计算） */
void feedback_update(struct feedback *feedback, float dt)
{
	if (!feedback || feedback->state != FEEDBACK_STATE_OK) {
		return;
	}

	struct feedback_param *param = feedback->param;
	struct feedback_data *data = &feedback->data;

	/* 1. 读取原始编码器值 */
	uint16_t raw = feedback_get_raw(feedback);
	data->raw = raw;

	/* 2. 应用零位偏移 */
	int32_t adjusted_raw = (int32_t)raw - (int32_t)param->encoder_offset;

	/* 3. 计算累积机械角度（处理溢出） */
	feedback_calc_accumulated_mangle(feedback, adjusted_raw);
	float cur_mangle = data->accumulated_mangle_rad;

	/* 4. 计算电角度 */
	feedback->output.eangle_rad = feedback_calc_elec_angle(feedback, cur_mangle);

	/* 5. 差分法计算机械角速度 */
	feedback->output.velocity_rad_s = feedback_calc_velocity(feedback, dt, cur_mangle);

	/* 6. 更新里程（可选） */
	feedback->output.odometer = cur_mangle * param->wheel_radius / param->gear_ratio;
}
