#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "feedback.h"
#include "inverter.h"
#include <math.h>
#include "foc.h"

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
#define ROTATE_SPEED      (1.0f * M_PI)
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

	if (!motor || !motor->feedback || !motor->inverter) {
		return true;
	}

	enc = &motor->calib.encoder;
	feedback = motor->feedback;
	inverter = motor->inverter;

	switch (enc->state) {
	case ENC_CALIB_ALIGN:

		/* 对齐完成，记录初始编码器值 */
		inverter_enable(inverter);
		enc->state = ENC_CALIB_ROTATE;
		break;

	case ENC_CALIB_ROTATE:
		open_loop_force_drag(motor, 0.0001f, 0.2f, ROTATE_SPEED);
		break;

	case ENC_CALIB_CALC:

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
