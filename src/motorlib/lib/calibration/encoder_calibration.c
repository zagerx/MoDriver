#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "feedback.h"
#include "inverter.h"
#include "currsmp.h"
#include "foc.h"
#include "svpwm.h"
#include <math.h>
#include "motorlib_control_param.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI (2.0f * M_PI)
#endif

/* 解卷绕辅助函数 - 处理编码器溢出 */
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

void encoder_calib_init(struct motor *motor)
{
	struct encoder_calib *enc;

	if (!motor) {
		return;
	}

	enc = &motor->calib.encoder;

	enc->tick_cnt = 0;
	enc->state = ENC_CALIB_ALIGN;
	enc->raw_prev = 0;
	enc->raw_delta_acc = 0;
	enc->align_tick_cnt = 0;

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
	uint16_t current_raw;
	int32_t delta;
	float mech_rounds, elec_rounds, ratio;
	int pole_pairs;
	int direction;
	float sum_eangle;
	if (!motor || !motor->feedback || !motor->inverter || !motor->currsmp) {
		return true;
	}

	enc = &motor->calib.encoder;
	feedback = motor->feedback;
	inverter = motor->inverter;

	switch (enc->state) {
	case ENC_CALIB_ALIGN:
		/* 第一次对齐：施加d轴电压对齐到0度电角度 */
		if (enc->align_tick_cnt == 0) {
			inverter_enable(inverter);
		}
		open_loop_force_align(motor, ALIGN_VOLTAGE, 0.0f);
		enc->align_tick_cnt++;

		if (enc->align_tick_cnt >= ALIGN_TIMEOUT_TICKS) {
			/* 对齐完成，记录初始编码器值 */
			enc->raw_prev = feedback_get_raw(feedback);
			enc->raw_delta_acc = 0;
			enc->tick_cnt = 0;
			enc->state = ENC_CALIB_ROTATE;
		}
		break;

	case ENC_CALIB_ROTATE:
		/* 开环旋转：以固定电角速度旋转，同时累加电角度和编码器变化 */
		open_loop_force_drag(motor, CONTROL_PERIOD_DT, ROTATE_VOLTAGE, ROTATE_SPEED);

		/* 读取编码器并解卷绕累加 */
		current_raw = feedback_get_raw(feedback);
		delta = unwrap_delta(current_raw, &enc->raw_prev);
		enc->raw_delta_acc += delta;

		enc->tick_cnt++;

		if (enc->tick_cnt >= ROTATE_TIMEOUT_TICKS) {
			/* 旋转完成，再读取一次确保最后一点也被记录 */
			current_raw = feedback_get_raw(feedback);
			delta = unwrap_delta(current_raw, &enc->raw_prev);
			enc->raw_delta_acc += delta;
			enc->state = ENC_CALIB_CALC;
		}
		break;

	case ENC_CALIB_CALC:
		/* 计算极对数和方向 */
		sum_eangle = open_loop_get_force_angle(motor);
		mech_rounds = (float)enc->raw_delta_acc / ENCODER_RESOLUTION_F;
		elec_rounds = sum_eangle / M_TWOPI;

		/* 检查阈值 - 确保转够圈数 */
		if (fabsf(mech_rounds) < MIN_MECH_ROUNDS || fabsf(elec_rounds) < MIN_ELEC_ROUNDS) {
			enc->state = ENC_CALIB_ERROR;
			inverter_disable(inverter);
			return true;
		}

		/* 计算极对数 */
		ratio = fabsf(elec_rounds / mech_rounds);
		pole_pairs = (int)roundf(ratio);
		if (pole_pairs < 1) {
			pole_pairs = 1;
		}

		/* 判断方向：电角度和机械角度变化是否同向 */
		if ((sum_eangle > 0 && enc->raw_delta_acc > 0) ||
		    (sum_eangle < 0 && enc->raw_delta_acc < 0)) {
			direction = 1;
		} else {
			direction = -1;
		}

		/* 更新反馈参数 */
		_feedback_update_param_pole_pairs(feedback, (float)pole_pairs);
		_feedback_update_param_direction(feedback, (float)direction);
		enc->state = ENC_CALIB_OFFSET_ALIGN;
		break;

	case ENC_CALIB_OFFSET_ALIGN:
		/* 第二次对齐：对齐到-90度，此时的编码器值即为偏置 */
		open_loop_force_align(motor, ALIGN_VOLTAGE, -M_PI / 2.0f);
		enc->align_tick_cnt++;

		if ((enc->align_tick_cnt * CONTROL_PERIOD_DT * 1000) >= ALIGN_TIMEOUT) {
			/* 对齐完成，读取编码器值作为偏置 */
			uint16_t offset = feedback_get_raw(feedback);
			_feedback_update_param_encoder_offset(feedback, offset);
			_feedback_update_param_encoder_resolution(feedback, ENCODER_RESOLUTION);

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
	/* 结果已在校准过程中实时应用，此函数保留用于后续扩展 */
	(void)motor;
}
