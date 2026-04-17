
/**
 * @file feedback.c
 * @brief 编码器反馈模块实现
 * @details 实现编码器绑定、初始化、溢出处理、角度/速度/里程计算
 */

#include <stdint.h>
#include <math.h>

#include "feedback.h"
#include "motorlib_constants.h"

/** @brief 低通滤波系数（0~1，越大响应越快） */
#define VELOCITY_LPF_ALPHA 0.08f

#if FEEDBACK_USE_PLL
/**
 * @brief 整数取模运算，确保结果在 [0, m-1] 范围内
 * @param[in] a 被除数
 * @param[in] m 除数（正数）
 * @return 取模结果
 */
static int32_t mod(int32_t a, int32_t m)
{
	int32_t r = a % m;
	if (r < 0) {
		r += m;
	}
	return r;
}

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
 * @brief 更新PLL增益（与ODrive保持一致）
 * @param[in] feedback 反馈实例
 * @param[in] dt 控制周期，单位：s
 */
static void feedback_update_pll_gains(struct feedback *feedback, float dt)
{
	struct feedback_data *data = &feedback->data;

	/* ODrive公式：pll_kp = 2.0f * bandwidth */
	data->pll_kp = 2.0f * FEEDBACK_PLL_BANDWIDTH;

	/* 临界阻尼：pll_ki = 0.25f * (pll_kp * pll_kp) */
	data->pll_ki = 0.25f * (data->pll_kp * data->pll_kp);

	/* 检查离散时间近似是否稳定（与ODrive相同检查） */
	if (!(dt * data->pll_kp < 1.0f)) {
		/* 增益不稳定，使用保守值 */
		data->pll_kp = 0.9f / dt;
		data->pll_ki = 0.25f * (data->pll_kp * data->pll_kp);
	}
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
#endif /* FEEDBACK_USE_PLL */

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
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops)
{
	if (feedback) {
		feedback->ops = ops;
	}
}

/**
 * @brief 绑定反馈参数
 * @param[in] feedback 反馈实例
 * @param[in] param 反馈参数
 * @return 无
 */
void feedback_bind_encoder_param(struct feedback *feedback, struct feedback_param *param)
{
	if (feedback) {
		feedback->param = param;
	}
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
	data->odometer_offset_mangle = 0.0f; /* 初始无偏移 */

#if FEEDBACK_USE_PLL
	/* 初始化PLL状态 */
	const int32_t cpr = (int32_t)param->encoder_resolution;
	data->pos_estimate_counts = 0.0f;
	data->pos_cpr_counts = 0.0f;
	data->vel_estimate_counts = 0.0f;
	data->delta_pos_cpr_counts = 0.0f;

	/* 初始化PLL增益（需要dt，但此时未知，暂设为0） */
	data->pll_kp = 0.0f;
	data->pll_ki = 0.0f;
#endif

	feedback->output.eangle_rad = 0.0f;
	feedback->output.velocity_rad_s = 0.0f;
	feedback->output.odometer = 0.0f;

	return FEEDBACK_ERROR_NONE;
}

/**
 * @brief 计算累积机械角度，处理编码器溢出
 * @param[in] feedback 反馈实例
 * @param[in] raw 原始编码器读数
 * @param[in] adjusted_raw 偏移校正后的编码器读数（未使用，保留用于接口兼容）
 * @return 无
 * @details 检测越过CPR边界的溢出并计算累积机械角度
 * @note 使用原始值计算差分，偏移量在角度计算时处理，避免负数存入uint16_t
 */
static void feedback_calc_accumulated_mangle(struct feedback *feedback, uint16_t raw,
					     int32_t adjusted_raw)
{
	struct feedback_param *param = feedback->param;
	struct feedback_data *data = &feedback->data;

	const float two_pi = MOTORLIB_TWOPI;
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 计算原始差值（使用原始值，偏移在差分中抵消） */
	int32_t delta = (int32_t)raw - (int32_t)data->prev_raw;

	/* 处理越过CPR边界的溢出 */
	if (delta > cpr / 2) {
		delta -= cpr;
	} else if (delta < -cpr / 2) {
		delta += cpr;
	}

	/* 更新累积计数 */
	data->total_counts += delta;

	/* 计算累积机械角度（考虑编码器零位偏移） */
	data->accumulated_mangle_rad =
		(two_pi / (float)cpr) *
		(float)(data->total_counts - (int32_t)param->encoder_offset) * param->direction;

	/* 保存原始值供下次使用（避免负数存入uint16_t导致的抖动问题） */
	data->prev_raw = raw;

	(void)adjusted_raw; /* 显式标记未使用，避免编译器警告 */
}

/**
 * @brief 计算电角度（机械角度 * 极对数，归一化到 [0, 2π]）
 * @param[in] feedback 反馈实例
 * @param[in] mangle 机械角度，单位：rad
 * @return float 电角度，单位：rad
 */
static float feedback_calc_elec_angle(struct feedback *feedback, float mangle)
{
	struct feedback_param *param = feedback->param;
	float eangle = mangle * param->pole_pairs;
	return normalize_angle(eangle);
}

/**
 * @brief 差分法计算机械角速度，带低通滤波
 * @param[in] feedback 反馈实例
 * @param[in] dt 采样周期，单位：s
 * @param[in] cur_mangle 当前机械角度，单位：rad
 * @return float 机械角速度，单位：rad/s
 * @note 使用差分法计算速度，并应用一阶低通滤波
 */
static float feedback_calc_velocity(struct feedback *feedback, float dt, float cur_mangle)
{
	struct feedback_data *data = &feedback->data;

	if (dt <= 0.0f) {
		return data->mech_omega_rad_s;
	}

	/* 计算角度差（处理跨越 2π 边界） */
	float dtheta = cur_mangle - data->prev_mangle_rad;
	if (dtheta > MOTORLIB_PI) {
		dtheta -= MOTORLIB_TWOPI;
	} else if (dtheta < -MOTORLIB_PI) {
		dtheta += MOTORLIB_TWOPI;
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

/**
 * @brief 更新反馈原始数据
 * @param[in] feedback 反馈实例
 * @return 无
 */
void feedback_update_raw(struct feedback *feedback)
{
	if (!feedback || feedback->state != FEEDBACK_STATE_OK) {
		return;
	}

	feedback->data.raw = feedback_get_raw(feedback);
}

/**
 * @brief 更新反馈数据（编码器读取 + 角度/速度/电角度计算）
 * @param[in] feedback 反馈实例
 * @param[in] dt 采样周期，单位：s
 * @return 无
 * @details 执行编码器读取、零位偏移校正、累积角度计算、电角度计算、速度计算和里程更新
 */
void feedback_update(struct feedback *feedback, float dt)
{
	if (!feedback || feedback->state != FEEDBACK_STATE_OK) {
		return;
	}

	struct feedback_param *param = feedback->param;
	struct feedback_data *data = &feedback->data;

	/* 1. 读取原始编码器值 */
	uint16_t raw = data->raw;

	/* 2. 应用零位偏移 */
	int32_t adjusted_raw = (int32_t)raw - (int32_t)param->encoder_offset;

	/* 3. 计算累积机械角度（处理溢出） */
	feedback_calc_accumulated_mangle(feedback, raw, adjusted_raw);
	float cur_mangle = data->accumulated_mangle_rad;

	/* 4. 计算电角度 */
	feedback->output.eangle_rad = feedback_calc_elec_angle(feedback, cur_mangle);

	/* 5. 计算机械角速度（根据宏选择PLL或差分法） */
#if FEEDBACK_USE_PLL
	/* 使用PLL（与ODrive保持一致） */
	const int32_t cpr = (int32_t)param->encoder_resolution;

	/* 第一次运行时初始化PLL增益 */
	if (data->pll_kp == 0.0f && data->pll_ki == 0.0f) {
		feedback_update_pll_gains(feedback, dt);
	}

	/* 计算当前CPR内的计数（0~CPR-1） */
	int32_t count_in_cpr = data->total_counts % cpr;
	if (count_in_cpr < 0) {
		count_in_cpr += cpr;
	}

	/* 运行PLL更新 */
	feedback->output.velocity_rad_s =
		feedback_pll_update(feedback, dt, count_in_cpr, data->total_counts);

	/* 更新机械角速度状态（保持兼容性） */
	data->mech_omega_rad_s = feedback->output.velocity_rad_s;
#else
	/* 使用差分法（原有实现） */
	feedback->output.velocity_rad_s = feedback_calc_velocity(feedback, dt, cur_mangle);
#endif

	feedback->output.line_velocity_mm_s =
		feedback->output.velocity_rad_s * param->wheel_radius / param->gear_ratio;

	/* 6. 应用小数偏移到电角度输出	小数偏移用于子计数精度的相位对齐 */
	if (param->encoder_offset_frac != 0.0f) {
		/* 将小数偏移转换为电角度弧度 */
		float frac_eangle_offset = param->encoder_offset_frac * MOTORLIB_TWOPI /
					   (float)param->encoder_resolution * param->pole_pairs;
		feedback->output.eangle_rad =
			normalize_angle(feedback->output.eangle_rad - frac_eangle_offset);
	}

	/* 7. 更新里程（应用偏移，实现相对零点） */
	/* 使用相对角度计算里程，支持Home模式下动态重置而不破坏角度连续性 */
	float relative_mangle = data->accumulated_mangle_rad - data->odometer_offset_mangle;
	feedback->output.odometer = relative_mangle * param->wheel_radius / param->gear_ratio;
}
