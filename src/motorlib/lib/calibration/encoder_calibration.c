#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "feedback.h"
#include "inverter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#undef M_TWOPI
#define M_TWOPI           (2.0f * M_PI)
#define ENCODER_MAX_COUNT 16383.0f
#define MIN_MECH_ROUNDS   0.02f
#define MIN_ELEC_ROUNDS   0.20f
#define ALIGN_VOLTAGE     0.05f
#define ROTATE_VOLTAGE    0.05f
#define ROTATE_SPEED      (2.0f * M_PI)
#define ALIGN_TICKS       1000
#define ROTATE_TICKS      30000

/* 解卷绕辅助函数 */
static inline int32_t unwrap_delta(uint32_t current, uint32_t *prev)
{
	int32_t diff = (int32_t)current - (int32_t)(*prev);
	int32_t half = (int32_t)(ENCODER_MAX_COUNT / 2.0f);

	if (diff > half) {
		diff -= (int32_t)ENCODER_MAX_COUNT + 1;
	} else if (diff < -half) {
		diff += (int32_t)ENCODER_MAX_COUNT + 1;
	}

	*prev = current;
	return diff;
}

void encoder_calib_init(struct motor *motor)
{
	struct encoder_calib *enc;

	if (!motor) {
		return;
	}

	enc = &motor->calib.encoder;

	enc->tick_cnt = 0;
	enc->state = ENC_CALIB_ALIGN;
	enc->elec_angle_acc = 0.0f;
	enc->raw_prev = 0;
	enc->raw_delta_acc = 0;
	enc->align_tick_target = ALIGN_TICKS;
	enc->align_tick_cnt = 0;
	enc->align_angle = 0.0f;
	enc->align_voltage = ALIGN_VOLTAGE;
	enc->rotate_voltage = ROTATE_VOLTAGE;
	enc->rotate_speed = ROTATE_SPEED;
	enc->rotate_tick_target = ROTATE_TICKS;

	/* 禁用逆变器准备开始 */
	if (motor->inverter) {
		inverter_disable(motor->inverter);
	}
}

bool encoder_calib_run(struct motor *motor)
{
	struct encoder_calib *enc;
	struct feedback *feedback;
	struct inverter *inverter;
	uint32_t current_raw;
	int32_t delta;
	float mech_rounds, elec_rounds, ratio;
	int pole_pairs;
	int direction;
	float u_d, u_q, u_alpha, u_beta, uu, uv, uw;
	float theta;

	if (!motor || !motor->feedback || !motor->inverter) {
		return true;
	}

	enc = &motor->calib.encoder;
	feedback = motor->feedback;
	inverter = motor->inverter;

	switch (enc->state) {
	case ENC_CALIB_ALIGN:
		/* 施加固定电压对齐到0度（Id=align_voltage, Iq=0） */
		if (enc->align_tick_cnt == 0) {
			uu = enc->align_voltage;
			uv = -0.5f * enc->align_voltage;
			uw = -0.5f * enc->align_voltage;
			inverter_set_voltage(inverter, uu, uv, uw);
		}

		enc->align_tick_cnt++;

		if (enc->align_tick_cnt >= enc->align_tick_target) {
			/* 对齐完成，记录初始编码器值 */
			enc->raw_prev = feedback_get_raw(feedback);
			enc->state = ENC_CALIB_ROTATE;
			enc->tick_cnt = 0;
		}
		break;

	case ENC_CALIB_ROTATE:
		/* 开环旋转：以固定电角速度旋转 */
		theta = enc->rotate_speed * ((float)enc->tick_cnt / 10000.0f);
		u_d = enc->rotate_voltage * cosf(theta);
		u_q = enc->rotate_voltage * sinf(theta);
		u_alpha = u_d;
		u_beta = u_q;
		/* 逆Park变换到三相 */
		uu = u_alpha;
		uv = -0.5f * u_alpha + 0.866f * u_beta;
		uw = -0.5f * u_alpha - 0.866f * u_beta;
		inverter_set_voltage(inverter, uu, uv, uw);

		/* 累加电角度 */
		enc->elec_angle_acc += enc->rotate_speed * 0.0001f;

		/* 解卷绕累加编码器 */
		current_raw = feedback_get_raw(feedback);
		delta = unwrap_delta(current_raw, &enc->raw_prev);
		enc->raw_delta_acc += delta;

		enc->tick_cnt++;

		if (enc->tick_cnt >= enc->rotate_tick_target) {
			/* 旋转完成，记录最终值 */
			current_raw = feedback_get_raw(feedback);
			delta = unwrap_delta(current_raw, &enc->raw_prev);
			enc->raw_delta_acc += delta;
			enc->state = ENC_CALIB_CALC;
		}
		break;

	case ENC_CALIB_CALC:
		/* 计算极对数和方向 */
		mech_rounds = (float)enc->raw_delta_acc / ENCODER_MAX_COUNT;
		elec_rounds = enc->elec_angle_acc / M_TWOPI;

		/* 检查阈值 */
		if (fabsf(mech_rounds) < MIN_MECH_ROUNDS || fabsf(elec_rounds) < MIN_ELEC_ROUNDS) {
			enc->state = ENC_CALIB_ERROR;
			inverter_disable(inverter);
			return true;
		}

		ratio = fabsf(elec_rounds / mech_rounds);
		pole_pairs = (int)roundf(ratio);

		/* 判断方向 */
		if ((enc->elec_angle_acc > 0 && enc->raw_delta_acc > 0) ||
		    (enc->elec_angle_acc < 0 && enc->raw_delta_acc < 0)) {
			direction = 1;
		} else {
			direction = -1;
		}

		/* 应用极对数和方向到feedback */
		_feedback_update_param_pole_pairs(feedback, (float)pole_pairs);
		_feedback_update_param_direction(feedback, (float)direction);

		/* 同时更新 motor_config */
		motor->config->pairs = (uint16_t)pole_pairs;

		/* 准备偏移校准 */
		enc->align_angle = -M_PI / 2.0f;
		enc->align_tick_cnt = 0;
		enc->state = ENC_CALIB_OFFSET_ALIGN;
		break;

	case ENC_CALIB_OFFSET_ALIGN:
		/* 对齐到-90度取偏置 */
		if (enc->align_tick_cnt == 0) {
			u_d = enc->align_voltage * cosf(enc->align_angle);
			u_q = enc->align_voltage * sinf(enc->align_angle);
			/* 逆Park变换到三相 */
			u_alpha = u_d;
			u_beta = u_q;
			uu = u_alpha;
			uv = -0.5f * u_alpha + 0.866f * u_beta;
			uw = -0.5f * u_alpha - 0.866f * u_beta;
			inverter_set_voltage(inverter, uu, uv, uw);
		}

		enc->align_tick_cnt++;

		if (enc->align_tick_cnt >= enc->align_tick_target * 2) {
			/* 读取编码器值作为偏置 */
			uint16_t offset = feedback_get_raw(feedback);
			_feedback_update_param_encoder_offset(feedback, offset);
			_feedback_update_param_encoder_resolution(feedback, 16384);

			enc->state = ENC_CALIB_DONE;
			inverter_disable(inverter);
			return true;
		}
		break;

	case ENC_CALIB_DONE:
		inverter_disable(inverter);
		return true;

	case ENC_CALIB_ERROR:
	default:
		inverter_disable(inverter);
		return true;
	}

	return false;
}

void encoder_calib_apply(struct motor *motor)
{
	/* 结果已在 ENC_CALIB_CALC 阶段实时应用 */
	(void)motor;
}
