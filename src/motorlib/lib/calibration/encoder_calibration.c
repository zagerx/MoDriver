
/**
 * @file encoder_calibration.c
 * @brief 编码器校准实现
 * @details 通过双向电角度扫描检测编码器方向、极对数及零位偏移
 */

#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "feedback.h"
#include "inverter.h"
#include "foc.h"
#include "open_loop.h"
#include <math.h>
#include "motorlib_control_param.h"

#undef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI (2.0f * M_PI)
#endif

/* 编码器校准配置参数 */
#define ENC_CALIB_SCAN_DISTANCE       (14.0f * M_PI) /* 扫描电角度距离，默认16π (8圈电角度) */
#define ENC_CALIB_SCAN_OMEGA          (2.0f * M_PI)  /* 扫描电角速度，默认8π rad/s (4圈/秒) */
#define ENC_CALIB_ALIGN_TIME          1.0f           /* 对齐保持时间，秒 */
#define ENC_CALIB_DIRECTION_THRESHOLD 8              /* 方向检测最小编码器变化计数 */
#define ENC_CALIB_CPR_TOLERANCE       0.02f          /* CPR校验容差，2% */

/* 预计算的tick阈值 */
#define ENC_CALIB_ALIGN_TICKS ((uint32_t)(ENC_CALIB_ALIGN_TIME / CONTROL_PERIOD_DT))

/**
 * @brief 解卷绕辅助函数 - 处理编码器溢出
 * @param[in] current 当前编码器值
 * @param[in,out] prev 上一次的编码器值指针
 * @return 解卷绕后的差值
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
 * @param[in] angle 输入角度
 * @return 归一化后的角度
 */
static inline float normalize_angle_0_2pi(float angle)
{
	angle = fmodf(angle, M_TWOPI);
	if (angle < 0.0f) {
		angle += M_TWOPI;
	}
	return angle;
}

/**
 * @brief 初始化编码器校准
 * @param[in] motor 电机实例
 */
void encoder_calib_init(struct motor *motor)
{
	struct encoder_calib *enc;

	if (!motor) {
		return;
	}

	enc = &motor->calib.encoder;

	/* 清零所有状态 */
	enc->tick_cnt = 0;
	enc->state = ENC_CALIB_ALIGN_START;
	enc->raw_prev = 0;
	enc->raw_delta_acc = 0;
	enc->align_tick_cnt = 0;
	enc->encvaluesum = 0;
	enc->num_steps = 0;
	enc->init_enc_val = 0;
	enc->calib_start_eangle = 0.0f;

	/* 禁用逆变器准备开始 */
	if (motor->inverter) {
		inverter_disable(motor->inverter);
	}
}

/**
 * @brief 执行一次编码器校准步进
 * @param[in] motor 电机实例
 * @return true 校准完成，false 需要继续执行
 * @note 应在高频任务中周期性调用
 */
bool encoder_calib_run(struct motor *motor)
{
	struct encoder_calib *enc;
	struct feedback *feedback;
	struct inverter *inverter;
	uint16_t current_raw;
	int32_t delta;
	float current_eangle;

	if (!motor || !motor->feedback || !motor->inverter || !motor->currsmp) {
		return true;
	}

	enc = &motor->calib.encoder;
	feedback = motor->feedback;
	inverter = motor->inverter;

	switch (enc->state) {
	case ENC_CALIB_ALIGN_START:
		/* 首次对齐：施加d轴电压对齐到起始电角度（0 - scan_distance/2） */
		if (enc->align_tick_cnt == 0) {
			inverter_enable(inverter);
			/* 计算起始电角度：从 -scan_distance/2 开始 */
			enc->calib_start_eangle = -ENC_CALIB_SCAN_DISTANCE / 2.0f;
			motor->foc.self_eangle = enc->calib_start_eangle;
		}

		/* 使用开环强制对齐到起始位置 */
		open_loop_force_align(motor, ALIGN_VOLTAGE, motor->foc.self_eangle);
		enc->align_tick_cnt++;

		if (enc->align_tick_cnt >= ENC_CALIB_ALIGN_TICKS) {
			/* 对齐完成，记录初始编码器值 */
			enc->init_enc_val = (int32_t)feedback_get_raw(feedback);
			enc->raw_prev = feedback_get_raw(feedback);
			enc->raw_delta_acc = 0;
			enc->encvaluesum = 0;
			enc->num_steps = 0;
			enc->tick_cnt = 0;
			enc->align_tick_cnt = 0;
			enc->state = ENC_CALIB_SCAN_FORWARD;
		}
		break;

	case ENC_CALIB_SCAN_FORWARD:
		/* 正向扫描：以固定电角速度旋转，同时累加编码器变化 */
		open_loop_force_drag(motor, CONTROL_PERIOD_DT, ALIGN_VOLTAGE, ENC_CALIB_SCAN_OMEGA);

		/* 读取编码器并累加 */
		current_raw = feedback_get_raw(feedback);
		delta = unwrap_delta(current_raw, &enc->raw_prev);
		enc->raw_delta_acc += delta;
		enc->encvaluesum += (int64_t)(enc->init_enc_val + enc->raw_delta_acc);
		enc->num_steps++;

		/* 检查是否达到扫描距离 */
		current_eangle = open_loop_get_force_angle(motor);
		if ((current_eangle - enc->calib_start_eangle) >= ENC_CALIB_SCAN_DISTANCE) {
			enc->state = ENC_CALIB_CHECK_RESPONSE;
		}
		break;

	case ENC_CALIB_CHECK_RESPONSE:
		/* 检查编码器响应和方向 */
		{
			int32_t enc_delta = enc->raw_delta_acc;
			float detected_direction;
			/* 检查是否有足够响应 */
			if (ABS(enc_delta) < ENC_CALIB_DIRECTION_THRESHOLD) {
				enc->state = ENC_CALIB_ERROR_NO_RESPONSE;
				inverter_disable(inverter);
				return true;
			}

			/* 判断方向 */
			if (enc_delta > 0) {
				detected_direction = 1;
			} else {
				detected_direction = -1;
			}

			/* 计算极对数 */
			float mech_rounds = (float)enc_delta / ENCODER_RESOLUTION_F;
			float elec_rounds = ENC_CALIB_SCAN_DISTANCE / M_TWOPI;
			float ratio = fabsf(elec_rounds / mech_rounds);
			int pole_pairs = (int)roundf(ratio);
			if (pole_pairs < 1) {
				pole_pairs = 1;
			}

			/* CPR校验：检查实际编码器变化与期望是否匹配 */
			float expected_enc_delta = (ENC_CALIB_SCAN_DISTANCE / M_TWOPI) *
						   (ENCODER_RESOLUTION_F / (float)pole_pairs);
			float actual_enc_delta = fabsf((float)enc_delta);
			float cpr_error =
				fabsf(actual_enc_delta - expected_enc_delta) / expected_enc_delta;

			if (cpr_error > ENC_CALIB_CPR_TOLERANCE) {
				/* CPR不匹配，可能是极对数错误或编码器问题 */
				enc->state = ENC_CALIB_ERROR_CPR_MISMATCH;
				inverter_disable(inverter);
				return true;
			}

			/* 更新反馈参数 */
			_feedback_update_param_pole_pairs(feedback, (float)pole_pairs);
			_feedback_update_param_direction(feedback, (float)detected_direction);

			/* 准备反向扫描 */
			enc->tick_cnt = 0;
			enc->state = ENC_CALIB_SCAN_BACKWARD;
		}
		break;

	case ENC_CALIB_SCAN_BACKWARD:
		/* 反向扫描：以相同速度反向旋转回起点 */
		open_loop_force_drag(motor, CONTROL_PERIOD_DT, ALIGN_VOLTAGE,
				     -ENC_CALIB_SCAN_OMEGA);

		/* 读取编码器并累加 */
		current_raw = feedback_get_raw(feedback);
		delta = unwrap_delta(current_raw, &enc->raw_prev);
		enc->raw_delta_acc += delta;
		enc->encvaluesum += (int64_t)(enc->init_enc_val + enc->raw_delta_acc);
		enc->num_steps++;

		/* 检查是否回到起始电角度 */
		current_eangle = open_loop_get_force_angle(motor);
		if ((current_eangle - enc->calib_start_eangle) <= 0.0f) {
			enc->state = ENC_CALIB_CALC_OFFSET;
		}
		break;

	case ENC_CALIB_CALC_OFFSET:
		/* 计算零点偏移（使用往返平均） */
		{
			int32_t offset_int;
			int64_t residual;
			float offset_frac;

			/* 整数部分：平均值 */
			offset_int = (int32_t)(enc->encvaluesum / (int64_t)enc->num_steps);

			/* 小数部分：余数平均 + 0.5f 中心对齐 */
			residual =
				enc->encvaluesum - ((int64_t)offset_int * (int64_t)enc->num_steps);
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
			_feedback_update_param_encoder_offset(feedback, (uint16_t)offset_int,
							      offset_frac);
			_feedback_update_param_encoder_resolution(feedback, ENCODER_RESOLUTION);

			enc->align_tick_cnt = 0;
			enc->state = ENC_CALIB_DONE;
		}
		break;

	case ENC_CALIB_DONE:
		inverter_disable(inverter);
		return true;

	case ENC_CALIB_ERROR_NO_RESPONSE:
	case ENC_CALIB_ERROR_CPR_MISMATCH:
	default:
		inverter_disable(inverter);
		return true;
	}

	return false;
}

/**
 * @brief 应用编码器校准结果
 * @param[in] motor 电机实例
 */
void encoder_calib_apply(struct motor *motor)
{
	/* 结果已在校准过程中实时应用，此函数保留用于后续扩展 */
	(void)motor;
}
