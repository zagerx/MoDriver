
/**
 * @file feedback.c
 * @brief 编码器反馈模块实现
 * @details 实现编码器绑定、初始化、溢出处理、角度/速度/里程计算
 */

#include <stdint.h>
#include <math.h>

#include "feedback.h"
#include "motor_interface_params.h"
#include "motorlib_constants.h"
#include "stdbool.h"
#include "_motorlib_internal.h"

/**
 * @brief 将角度包装到 [-π, π] 范围内
 * @param[in] angle 输入角度
 * @param[in] period 周期（通常为2π或CPR）
 * @return 包装后的角度
 */
static float wrap_pm(float angle, float period)
{
	angle = fmodf(angle + period / 2.0f, period);
	if (angle < 0.0f) {
		angle += period;
	}
	return angle - period / 2.0f;
}

/**
 * @brief 将角度包装到 [0, period] 范围内
 * @param[in] angle 输入角度
 * @param[in] period 周期
 * @return 包装后的角度
 */
static float fmodf_pos(float angle, float period)
{
	angle = fmodf(angle, period);
	if (angle < 0.0f) {
		angle += period;
	}
	return angle;
}

/**
 * @brief 编码器模型函数（简单向下取整，与ODrive保持一致）
 * @param[in] internal_pos 内部位置估计
 * @return 编码器计数
 */
static int32_t encoder_model(float internal_pos)
{
	return (int32_t)floorf(internal_pos);
}

/**
 * @brief PLL更新函数（与ODrive保持一致）
 * @param[in] feedback 反馈实例
 * @param[in] dt 采样周期，单位：s
 * @param[in] count_in_cpr 当前CPR内的编码器计数（0~CPR-1）
 * @param[in] shadow_count 阴影计数（连续计数，无溢出）
 * @return 速度估计（计数/秒）
 */
static float feedback_pll_update(struct feedback *feedback, float dt, int32_t count_in_cpr,
				 int32_t shadow_count)
{
	struct feedback_data *data = &feedback->data;
	struct feedback_param *param = feedback->param;
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 1. 预测当前位置 */
	data->pos_estimate_counts += dt * data->vel_estimate_counts;
	data->pos_cpr_counts += dt * data->vel_estimate_counts;

	/* 2. 离散相位检测器（与ODrive相同） */
	float delta_pos_counts = (float)(shadow_count - encoder_model(data->pos_estimate_counts));
	float delta_pos_cpr_counts = (float)(count_in_cpr - encoder_model(data->pos_cpr_counts));
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)cpr);

	/* 调试变量（与ODrive相同） */
	data->delta_pos_cpr_counts += 0.1f * (delta_pos_cpr_counts - data->delta_pos_cpr_counts);

	/* 3. PLL反馈（与ODrive相同） */
	data->pos_estimate_counts += dt * data->pll_kp * delta_pos_counts;
	data->pos_cpr_counts += dt * data->pll_kp * delta_pos_cpr_counts;
	data->pos_cpr_counts = fmodf_pos(data->pos_cpr_counts, (float)cpr);
	data->vel_estimate_counts += dt * data->pll_ki * delta_pos_cpr_counts;

	/* 4. 零速对齐（与ODrive相同，防止抖动） */
	if (fabsf(data->vel_estimate_counts) < 0.5f * dt * data->pll_ki) {
		data->vel_estimate_counts = 0.0f;
	}

	/* 5. 转换为机械角速度（rad/s） */
	float mech_omega_rad_s =
		data->vel_estimate_counts * (MOTORLIB_TWOPI / (float)cpr) * param->direction;

	return mech_omega_rad_s;
}
/**
 * @brief 角度归一化到 [0, 2PI]
 * @param[in] angle 输入角度，单位：rad
 * @return float 归一化后的角度，单位：rad
 * @note 将任意角度映射到 [0, 2π] 范围内
 */
static float normalize_angle(float angle)
{
	angle = fmodf(angle, MOTORLIB_TWOPI);
	if (angle < 0.0f) {
		angle += MOTORLIB_TWOPI;
	}
	return angle;
}

/**
 * @brief 绑定编码器操作接口
 * @param[in] feedback 反馈实例
 * @param[in] ops 编码器操作接口
 * @return 无
 */
void feedback_bind_encoder(struct motor *motor, const struct encoder_ops *ops)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = &motor->feedback;
	feedback->ops = ops;
}

/**
 * @brief 绑定反馈参数
 * @param[in] feedback 反馈实例
 * @param[in] param 反馈参数
 * @return 无
 */
void feedback_bind_encoder_param(struct motor *motor, struct feedback_param *param)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = &motor->feedback;
	feedback->param = param;
}

/**
 * @brief 初始化反馈模块，校验参数合法性
 * @param[in] feedback 反馈实例
 * @return feedback_error_code 错误码
 * @retval FEEDBACK_ERROR_NONE 初始化成功
 * @retval FEEDBACK_ERROR_PARAM 参数错误
 * @retval FEEDBACK_ERROR_HW_FAILURE 硬件故障
 * @note 校验轮子半径、减速比、极对数、方向、编码器分辨率等参数
 */
void feedback_init(struct motor *motor)
{
	if (!motor) {
		return;
	}

	struct feedback *feedback = &motor->feedback;
	if (!feedback->ops || !feedback->ops->read) {
		return;
	}

	struct feedback_data *data = &feedback->data;
	data->raw = 0;
	data->prev_raw = 0;
	data->total_counts = 0;
	data->odometer_offset_mangle = 0.0f; /* 初始无偏移 */
	data->phase_interp = 0.5f;           /* 静止时插值位于计数中点 */

	/* 初始化PLL状态 */
	data->pos_estimate_counts = 0.0f;
	data->pos_cpr_counts = 0.0f;
	data->vel_estimate_counts = 0.0f;
	data->delta_pos_cpr_counts = 0.0f;

	/* 初始化PLL增益（基于带宽一次性计算，运行时不改变） */
	data->pll_kp = 2.0f * FEEDBACK_PLL_BANDWIDTH;
	data->pll_ki = 0.25f * (data->pll_kp * data->pll_kp);

	feedback->output.eangle_rad = 0.0f;
	feedback->output.mangle_rad = 0.0f;
	feedback->output.velocity_rad_s = 0.0f;
	return;
}
void feedback_reset_encoder(struct motor *motor)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = &motor->feedback;
	struct feedback_param *param = feedback->param;
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 读取当前编码器值 */
	uint16_t raw = feedback_get_raw(feedback);
	int32_t count_in_cpr = (int32_t)raw % cpr;
	if (count_in_cpr < 0) {
		count_in_cpr += cpr;
	}

	/* 同步所有内部状态到当前位置（校准期间的位移全部丢弃） */
	feedback->data.raw = raw;
	feedback->data.prev_raw = raw;
	feedback->data.total_counts = count_in_cpr;
	feedback->data.pos_estimate_counts = (float)count_in_cpr;
	feedback->data.pos_cpr_counts = (float)count_in_cpr;
	feedback->data.vel_estimate_counts = 0.0f;
	feedback->data.phase_interp = 0.5f;

	/* 清零输出 */
	feedback->output.eangle_rad = 0.0f;
	feedback->output.mangle_rad = 0.0f;
	feedback->output.velocity_rad_s = 0.0f;

	/* 当前位置作为里程零点 */
	feedback->data.odometer_offset_mangle = 0.0f;
}
/**
 * @brief 更新反馈原始数据
 * @param[in] motor 电机实例
 * @return 无
 */
void feedback_update_raw(struct motor *motor)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = &motor->feedback;
	feedback->data.raw = feedback_get_raw(feedback);
}

/**
 * @brief 更新反馈数据（编码器读取 + 角度/速度/电角度计算）
 * @param[in] motor 电机实例
 * @param[in] dt 采样周期，单位：s
 * @return 无
 * @details 执行编码器读取、PLL速度估计、电角度插值计算和里程更新
 * @note 电角度采用ODrive方案：编码器整数计数 + PLL速度插值
 */
void feedback_update(struct motor *motor, float dt)
{
	if (!motor) {
		return;
	}
	struct feedback *feedback = &motor->feedback;

	struct motor_electrical_param *elec_param = &motor->param_ext->electrical_param;
	float pairs = elec_param->pole_pairs;
	struct feedback_param *param = feedback->param;
	struct feedback_data *data = &feedback->data;
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 1. 读取原始编码器值 */
	uint16_t raw = data->raw;

	/* 2. 计算 delta_enc（处理溢出，与ODrive一致） */
	int32_t delta_enc = (int32_t)raw - (int32_t)data->prev_raw;
	if (delta_enc > cpr / 2) {
		delta_enc -= cpr;
	} else if (delta_enc < -cpr / 2) {
		delta_enc += cpr;
	}

	/* 3. 更新 shadow_count */
	data->total_counts += delta_enc;
	data->prev_raw = raw;

	/* 4. 计算 CPR 内计数 */
	int32_t count_in_cpr = data->total_counts % cpr;
	if (count_in_cpr < 0) {
		count_in_cpr += cpr;
	}

	/* 5. 运行PLL */
	float mech_omega_rad_s =
		feedback_pll_update(feedback, dt, count_in_cpr, data->total_counts);
	feedback->output.velocity_rad_s = mech_omega_rad_s;

	/* 6. 电角度插值（ODrive方案） */
	bool snap_to_zero_vel = (data->vel_estimate_counts == 0.0f);
	int32_t corrected_enc = count_in_cpr - (int32_t)param->encoder_offset;

	if (snap_to_zero_vel) {
		/* 状态A：静止 — 插值锁定在计数中点，防止边界抖动 */
		data->phase_interp = 0.5f;
	} else {
		/* 非静止状态，判断编码器是否跳变 */
		if (delta_enc > 0) {
			/* 状态B：编码器正向跳变 — 以新计数为起点 */
			data->phase_interp = 0.0f;
		} else {
			if (delta_enc < 0) {
				/* 状态C：编码器反向跳变 — 以新计数为终点 */
				data->phase_interp = 1.0f;
			} else {
				/* 状态D：正常运行 — 用PLL速度预测插值 */
				data->phase_interp += dt * data->vel_estimate_counts;
				if (data->phase_interp > 1.0f) {
					data->phase_interp = 1.0f;
				} else if (data->phase_interp < 0.0f) {
					data->phase_interp = 0.0f;
				}
			}
		}
	}

	float interpolated_enc = (float)corrected_enc + data->phase_interp;

	/* 7. 计算电角度 */
	float elec_rad_per_enc = pairs * MOTORLIB_TWOPI / (float)cpr;
	float ph = elec_rad_per_enc * (interpolated_enc - param->encoder_offset_frac);
	feedback->output.eangle_rad = normalize_angle(ph) * param->direction;
	/* 8. 更新里程（使用PLL位置估计，扣除零位偏移） */
	float mangle_rad =
		(data->pos_estimate_counts - param->encoder_offset - param->encoder_offset_frac) *
		(MOTORLIB_TWOPI / (float)cpr) * param->direction;
	feedback->output.mangle_rad = mangle_rad;
}
