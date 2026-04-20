
/**
 * @file encoder_calibration.c
 * @brief 编码器校准实现
 * @details 通过双向电角度扫描检测编码器方向、极对数及零位偏移
 * 纯逻辑实现，不维护状态，由 calibration.c 统一状态管理
 */

#include "calibration.h"
#include "_motorlib_internal.h"
#include "feedback.h"
#include "inverter.h"
#include "foc.h"
#include "motor_interface_params.h"
#include "open_loop.h"
#include "motorlib_control_param.h"
#include "motorlib_constants.h"
#include <math.h>

#undef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))

/* ============ 配置参数 ============ */
#define ENC_CALIB_SCAN_DISTANCE       (14.0f * MOTORLIB_PI) /* 扫描电角度距离 */
#define ENC_CALIB_SCAN_OMEGA          (MOTORLIB_TWOPI)      /* 扫描电角速度 */
#define ENC_CALIB_ALIGN_TIME          (1.0f)                /* 对齐保持时间 [s] */
#define ENC_CALIB_DIRECTION_THRESHOLD (8)                   /* 方向检测最小编码器变化 */
#define ENC_CALIB_CPR_TOLERANCE       (0.02f)               /* CPR校验容差 2% */

/* 预计算的tick阈值 */
#define ENC_CALIB_ALIGN_TICKS ((uint32_t)(ENC_CALIB_ALIGN_TIME / CONTROL_PERIOD_DT))

/* ============ 内部辅助函数 ============ */

/**
 * @brief 解卷绕辅助函数 - 处理编码器溢出
 */
static inline int32_t unwrap_delta(uint16_t current, uint16_t *prev)
{
	int32_t diff = (int32_t)current - (int32_t)(*prev);
	int32_t half = (int32_t)(ENCODER_RESOLUTION / 2);

	if (diff > half) {
		diff -= (int32_t)ENCODER_RESOLUTION;
	} else if (diff < -half) {
		diff += (int32_t)ENCODER_RESOLUTION;
	}

	*prev = current;
	return diff;
}

/**
 * @brief 归一化角度到 [0, 2π)
 */
static inline float normalize_angle_0_2pi(float angle)
{
	angle = fmodf(angle, MOTORLIB_TWOPI);
	if (angle < 0.0f) {
		angle += MOTORLIB_TWOPI;
	}
	return angle;
}

/* ============ 接口实现 ============ */

/**
 * @brief 编码器校准准备
 * @param[in] motor 电机实例
 */
void enc_calib_prepare(struct motor *motor)
{
	struct encoder_calib_data *enc;
	struct inverter *inverter;

	if (!motor) {
		return;
	}

	enc = &motor->calib.enc;

	/* 清零所有数据 */
	enc->tick_cnt = 0;
	enc->raw_prev = 0;
	enc->raw_delta_acc = 0;
	enc->align_tick_cnt = 0;
	enc->encvaluesum = 0;
	enc->num_steps = 0;
	enc->init_enc_val = 0;
	enc->calib_start_eangle = 0.0f;
	enc->scan_delta = 0;

	/* 禁用逆变器准备开始 */
	inverter = &motor->inverter;
	if (inverter) {
		inverter_disable(inverter);
	}
}

/**
 * @brief 编码器对齐单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_align_step(struct motor *motor)
{
	struct encoder_calib_data *enc;
	struct inverter *inverter;

	if (!motor) {
		return 1;
	}

	enc = &motor->calib.enc;
	inverter = &motor->inverter;

	/* 首次进入：启用逆变器并设置起始角度 */
	if (enc->align_tick_cnt == 0) {
		inverter_enable(inverter);
		/* 计算起始电角度：从 -scan_distance/2 开始 */
		enc->calib_start_eangle = -ENC_CALIB_SCAN_DISTANCE / 2.0f;
		motor->foc.self_eangle = enc->calib_start_eangle;
	}

	/* 使用开环强制对齐到起始位置 */
	open_loop_force_align(motor, ALIGN_VOLTAGE, motor->foc.self_eangle);
	enc->align_tick_cnt++;

	if (enc->align_tick_cnt < ENC_CALIB_ALIGN_TICKS) {
		return 0; /* 继续 */
	}

	/* 对齐完成，记录初始编码器值 */
	enc->init_enc_val = (int32_t)feedback_get_raw(&motor->feedback);
	enc->raw_prev = feedback_get_raw(&motor->feedback);
	enc->raw_delta_acc = 0;
	enc->encvaluesum = 0;
	enc->num_steps = 0;
	enc->tick_cnt = 0;
	enc->align_tick_cnt = 0;

	return 1; /* 完成 */
}

/**
 * @brief 编码器正向扫描单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_scan_forward_step(struct motor *motor)
{
	struct encoder_calib_data *enc;
	struct feedback *feedback;
	uint16_t current_raw;
	int32_t delta;
	float current_eangle;

	if (!motor) {
		return 1;
	}

	enc = &motor->calib.enc;
	feedback = &motor->feedback;

	/* 正向扫描：以固定电角速度旋转 */
	open_loop_force_drag(motor, CONTROL_PERIOD_DT, ALIGN_VOLTAGE, ENC_CALIB_SCAN_OMEGA);

	/* 读取编码器并累加 */
	current_raw = feedback_get_raw(feedback);
	delta = unwrap_delta(current_raw, &enc->raw_prev);
	enc->raw_delta_acc += delta;
	enc->encvaluesum += (int64_t)(enc->init_enc_val + enc->raw_delta_acc);
	enc->num_steps++;

	/* 检查是否达到扫描距离 */
	current_eangle = open_loop_get_force_angle(motor);
	if ((current_eangle - enc->calib_start_eangle) < ENC_CALIB_SCAN_DISTANCE) {
		return 0; /* 继续 */
	}

	return 1; /* 完成 */
}

/**
 * @brief 检查编码器响应和计算极对数/方向
 * @param[in] motor 电机实例
 * @param[out] out_delta 输出扫描累计值
 * @return true=成功, false=失败（无响应或CPR不匹配）
 */
bool enc_check_response(struct motor *motor, int32_t *out_delta)
{
	struct encoder_calib_data *enc;
	struct feedback *feedback;
	int32_t enc_delta;
	float detected_direction;
	float mech_rounds, elec_rounds, ratio;
	int pole_pairs;
	float expected_enc_delta, actual_enc_delta, cpr_error;
	struct motor_param_ext *param_ext = motor->param_ext;
	if (!motor || !out_delta) {
		return false;
	}

	enc = &motor->calib.enc;
	feedback = &motor->feedback;

	enc_delta = enc->raw_delta_acc;

	/* 检查是否有足够响应 */
	if (ABS(enc_delta) < ENC_CALIB_DIRECTION_THRESHOLD) {
		inverter_disable(&motor->inverter);
		return false;
	}

	/* 判断方向 */
	detected_direction = (enc_delta > 0) ? 1.0f : -1.0f;

	/* 计算极对数 */
	mech_rounds = (float)enc_delta / ENCODER_RESOLUTION_F;
	elec_rounds = ENC_CALIB_SCAN_DISTANCE / MOTORLIB_TWOPI;
	ratio = fabsf(elec_rounds / mech_rounds);
	pole_pairs = (int)roundf(ratio);
	if (pole_pairs < 1) {
		pole_pairs = 1;
	}

	/* CPR校验 */
	expected_enc_delta = (ENC_CALIB_SCAN_DISTANCE / MOTORLIB_TWOPI) *
			     (ENCODER_RESOLUTION_F / (float)pole_pairs);
	actual_enc_delta = fabsf((float)enc_delta);
	cpr_error = fabsf(actual_enc_delta - expected_enc_delta) / expected_enc_delta;

	if (cpr_error > ENC_CALIB_CPR_TOLERANCE) {
		inverter_disable(&motor->inverter);
		return false;
	}

	/* 更新反馈参数 */

	// _feedback_update_param_pole_pairs(feedback, (float)pole_pairs);
	param_ext->electrical_param.pole_pairs = (float)pole_pairs;
	_feedback_update_param_direction(feedback, detected_direction);

	/* 准备反向扫描 */
	enc->tick_cnt = 0;
	*out_delta = enc_delta;

	return true;
}

/**
 * @brief 编码器反向扫描单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_scan_backward_step(struct motor *motor)
{
	struct encoder_calib_data *enc;
	struct feedback *feedback;
	uint16_t current_raw;
	int32_t delta;
	float current_eangle;

	if (!motor) {
		return 1;
	}

	enc = &motor->calib.enc;
	feedback = &motor->feedback;

	/* 反向扫描：以相同速度反向旋转 */
	open_loop_force_drag(motor, CONTROL_PERIOD_DT, ALIGN_VOLTAGE, -ENC_CALIB_SCAN_OMEGA);

	/* 读取编码器并累加 */
	current_raw = feedback_get_raw(feedback);
	delta = unwrap_delta(current_raw, &enc->raw_prev);
	enc->raw_delta_acc += delta;
	enc->encvaluesum += (int64_t)(enc->init_enc_val + enc->raw_delta_acc);
	enc->num_steps++;

	/* 检查是否回到起始电角度 */
	current_eangle = open_loop_get_force_angle(motor);
	if ((current_eangle - enc->calib_start_eangle) > 0.0f) {
		return 0; /* 继续 */
	}

	return 1; /* 完成 */
}

/**
 * @brief 计算编码器零点偏移
 * @param[in] motor 电机实例
 * @param[in] scan_delta 正向扫描累计值
 */
void enc_calc_offset(struct motor *motor, int32_t scan_delta)
{
	struct encoder_calib_data *enc;
	struct feedback *feedback;
	int32_t offset_int;
	int64_t residual;
	float offset_frac;

	if (!motor) {
		return;
	}

	enc = &motor->calib.enc;
	feedback = &motor->feedback;
	(void)scan_delta; /* 保留参数，与原逻辑一致 */

	/* 整数部分：平均值 */
	offset_int = (int32_t)(enc->encvaluesum / (int64_t)enc->num_steps);

	/* 小数部分：余数平均 + 0.5f 中心对齐 */
	residual = enc->encvaluesum - ((int64_t)offset_int * (int64_t)enc->num_steps);
	offset_frac = (float)residual / (float)enc->num_steps + 0.5f;

	/* 限制小数部分在合理范围 */
	if (offset_frac >= 1.0f) {
		offset_frac -= 1.0f;
		offset_int += 1;
	} else if (offset_frac < 0.0f) {
		offset_frac += 1.0f;
		offset_int -= 1;
	}

	/* 归一化到 [0, CPR) 范围 */
	while (offset_int < 0) {
		offset_int += ENCODER_RESOLUTION;
	}
	while (offset_int >= (int32_t)ENCODER_RESOLUTION) {
		offset_int -= ENCODER_RESOLUTION;
	}

	/* 更新反馈参数 */
	_feedback_update_param_encoder_offset(feedback, (uint16_t)offset_int, offset_frac);
	_feedback_update_param_encoder_resolution(feedback, ENCODER_RESOLUTION);
}

/**
 * @brief 禁用逆变器（校准结果已在enc_calc_offset中应用）
 * @param[in] motor 电机实例
 */
void enc_calib_apply(struct motor *motor)
{
	if (!motor) {
		return;
	}

	inverter_disable(&motor->inverter);
}
