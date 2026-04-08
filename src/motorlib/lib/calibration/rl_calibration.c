/**
 * @file rl_calibration.c
 * @brief 电机相电阻/相电感校准实现
 * @details 基于ODrive的校准算法，使用电流闭环测量电阻，方波电压测量电感
 */

#include "rl_calibration.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "open_loop.h"
#include "motorlib_control_param.h"
#include <math.h>

/* ============ 配置宏定义 ============ */
#define RL_RES_TEST_CURRENT (1.0f) /* 电阻测量目标电流 [A] */
#define RL_RES_MAX_VOLTAGE (2.0f) /* 电阻测量最大电压 [V] */
#define RL_RES_KI (1.0f) /* 电流环积分增益 */
#define RL_RES_SAMPLES (3000) /* 电阻测量采样次数 */

#define RL_IND_TEST_VOLTAGE (4.0f) /* 电感测量电压 [V]，提高到4V */
#define RL_IND_SAMPLES (1250) /* 电感测量采样次数 */

#define RL_RES_MIN_VALID (0.005f) /* 最小有效电阻 [Ohm] */
#define RL_RES_MAX_VALID (5.0f) /* 最大有效电阻 [Ohm] */
#define RL_IND_MIN_VALID (2e-6f) /* 最小有效电感 [H] */
#define RL_IND_MAX_VALID (4000e-6f) /* 最大有效电感 [H] */
#define RL_UNBALANCE_THRESHOLD (0.2f) /* 相不平衡阈值 */

/* ============ 内部函数 ============ */

/**
 * @brief 获取当前 I_alpha（直接计算，不调用 foc_update_idiq）
 */
static inline float get_I_alpha(struct motor *motor)
{
	struct currsmp *currsmp = &motor->currsmp;
	/* 直接计算：i_a = (raw - offset) * gain */
	float i_a = (currsmp->input.i_a_raw - currsmp->param->a_chn_offset) * currsmp->param->gain_phase;
	/* 固定在角度0，I_alpha = i_a */
	return i_a;
}

/**
 * @brief 获取当前 I_beta（直接计算）
 */
static inline float get_I_beta(struct motor *motor)
{
	struct currsmp *currsmp = &motor->currsmp;
	float i_a = (currsmp->input.i_a_raw - currsmp->param->a_chn_offset) * currsmp->param->gain_phase;
	float i_b = (currsmp->input.i_b_raw - currsmp->param->b_chn_offset) * currsmp->param->gain_phase;
	/* Clarke变换：I_beta = (i_a + 2*i_b) / sqrt(3) */
	return (i_a + 2.0f * i_b) * 0.577350269f;
}

/**
 * @brief 电阻测量阶段
 */
static bool resistance_phase_run(struct motor *motor)
{
	struct rl_calib *rl = &motor->calib.rl;

	/* 获取电流反馈 */
	float I_alpha = get_I_alpha(motor);
	float I_beta = get_I_beta(motor);

	/* 电流闭环积分 */
	float error = rl->current_setpoint - I_alpha;
	rl->voltage_accumulator += RL_RES_KI * CONTROL_PERIOD_DT * error;

	/* 限幅 */
	if (rl->voltage_accumulator > rl->voltage_limit) {
		rl->voltage_accumulator = rl->voltage_limit;
	} else if (rl->voltage_accumulator < -rl->voltage_limit) {
		rl->voltage_accumulator = -rl->voltage_limit;
	}

	/* 累加 I_beta 用于相平衡检测 */
	rl->I_beta_accumulator +=
		0.01f * CONTROL_PERIOD_DT * (I_beta - rl->I_beta_accumulator);

	/* 施加电压（固定角度0） */
	open_loop_force_align(motor, rl->voltage_accumulator, 0.0f);

	/* 调试标志：进入电阻测量，记录采样计数 */
	motor->data.debug.test_flag1 = 0x11;
	motor->data.debug.test_flag2 = rl->sample_cnt;

	rl->sample_cnt++;

	/* 检查完成 */
	if (rl->sample_cnt >= rl->target_samples) {
		inverter_disable(&motor->inverter);

		/* 计算电阻 R = V/I */
		if (fabsf(I_alpha) > 0.1f) {
			rl->measured_resistance = fabsf(rl->voltage_accumulator) / fabsf(I_alpha);
			/* 调试标志：电阻测量成功 */
			motor->data.debug.test_flag1 = 0x21;
			motor->data.debug.test_flag2 = (uint16_t)(rl->measured_resistance * 1000.0f);
		} else {
			/* 调试标志：错误，电流太小 */
			motor->data.debug.test_flag1 = 0xE1;
			motor->data.debug.test_flag2 = (uint16_t)(fabsf(I_alpha) * 1000.0f);
			rl->state = RL_CALIB_STATE_ERROR_RESISTANCE_OUT_OF_RANGE;
			return true;
		}

		/* 检查相不平衡 */
		if (fabsf(rl->I_beta_accumulator) / rl->current_setpoint > RL_UNBALANCE_THRESHOLD) {
			/* 调试标志：相不平衡错误 */
			motor->data.debug.test_flag1 = 0xE4;
			motor->data.debug.test_flag2 = (uint16_t)(fabsf(rl->I_beta_accumulator) * 1000.0f);
			rl->state = RL_CALIB_STATE_ERROR_UNBALANCED_PHASES;
			return true;
		}

		/* 检查范围 */
		if (rl->measured_resistance < RL_RES_MIN_VALID ||
		    rl->measured_resistance > RL_RES_MAX_VALID) {
			/* 调试标志：电阻超出范围 */
			motor->data.debug.test_flag1 = 0xE5;
			motor->data.debug.test_flag2 = (uint16_t)(rl->measured_resistance * 1000.0f);
			rl->state = RL_CALIB_STATE_ERROR_RESISTANCE_OUT_OF_RANGE;
			return true;
		}

		return true; /* 阶段完成 */
	}

	return false;
}

/**
 * @brief 电感测量阶段
 */
static bool inductance_phase_run(struct motor *motor)
{
	struct rl_calib *rl = &motor->calib.rl;

	/* 切换电压极性（方波） */
	rl->voltage_polarity = !rl->voltage_polarity;
	float voltage = rl->voltage_polarity ? rl->test_voltage : -rl->test_voltage;

	/* 施加电压 */
	open_loop_force_align(motor, voltage, 0.0f);

	/* 调试标志：进入电感测量 */
	motor->data.debug.test_flag1 = 0x12;
	motor->data.debug.test_flag2 = rl->sample_cnt;

	/* 测量电流 */
	float I_alpha = get_I_alpha(motor);

	if (rl->sample_cnt > 0) {
		float dI = I_alpha - rl->last_I_alpha;
		/* 累加电流变化的绝对值（电压切换导致的电流变化） */
		rl->delta_I_sum += fabsf(dI);
	}

	rl->last_I_alpha = I_alpha;
	rl->sample_cnt++;

	/* 检查完成 */
	if (rl->sample_cnt >= rl->target_samples) {
		inverter_disable(&motor->inverter);

		/* 计算电感 L = V / (dI/dt) */
		float dt = (float)rl->sample_cnt * CONTROL_PERIOD_DT;
		if (rl->delta_I_sum > 0.1f) {  /* 变化量必须足够大 */
			rl->measured_inductance = fabsf(rl->test_voltage) / (rl->delta_I_sum / dt);
			/* 调试标志：电感测量成功 */
			motor->data.debug.test_flag1 = 0x22;
			motor->data.debug.test_flag2 = (uint16_t)(rl->measured_inductance * 1000000.0f); /* uH */
		} else {
			/* 调试标志：电感错误，电流变化太小 */
			motor->data.debug.test_flag1 = 0xE2;
			motor->data.debug.test_flag2 = (uint16_t)(rl->delta_I_sum * 1000.0f);
			rl->state = RL_CALIB_STATE_ERROR_INDUCTANCE_OUT_OF_RANGE;
			return true;
		}

		/* 检查范围 */
		if (rl->measured_inductance < RL_IND_MIN_VALID ||
		    rl->measured_inductance > RL_IND_MAX_VALID) {
			/* 调试标志：电感超出范围 */
			motor->data.debug.test_flag1 = 0xE3;
			motor->data.debug.test_flag2 = (uint16_t)(rl->measured_inductance * 1000000.0f); /* uH */
			rl->state = RL_CALIB_STATE_ERROR_INDUCTANCE_OUT_OF_RANGE;
			return true;
		}

		/* 调试标志：RL校准全部完成 */
		motor->data.debug.test_flag1 = 0x30;
		motor->data.debug.test_flag2 = 0;
		rl->state = RL_CALIB_STATE_FINISH;
		return true;
	}

	return false;
}

/* ============ 接口函数 ============ */

void rl_calib_init(struct motor *motor)
{
	struct rl_calib *rl;

	if (!motor)
		return;

	rl = &motor->calib.rl;

	/* 初始化状态 */
	rl->state = RL_CALIB_STATE_RESISTANCE;
	rl->sample_cnt = 0;
	rl->target_samples = RL_RES_SAMPLES;

	/* 电阻测量参数 */
	rl->current_setpoint = RL_RES_TEST_CURRENT;
	rl->voltage_limit = RL_RES_MAX_VOLTAGE;
	rl->voltage_accumulator = 0.0f;
	rl->I_beta_accumulator = 0.0f;

	/* 电感测量参数 */
	rl->test_voltage = RL_IND_TEST_VOLTAGE;
	rl->last_I_alpha = 0.0f;
	rl->delta_I_sum = 0.0f;
	rl->voltage_polarity = false;

	/* 结果清零 */
	rl->measured_resistance = 0.0f;
	rl->measured_inductance = 0.0f;

	/* 调试标志：RL初始化，状态=RESISTANCE(1) */
	motor->data.debug.test_flag1 = 0x10;
	motor->data.debug.test_flag2 = 0;

	inverter_enable(&motor->inverter);
}

bool rl_calib_run(struct motor *motor)
{
	struct rl_calib *rl;
	bool done;

	if (!motor)
		return true;

	rl = &motor->calib.rl;

	switch (rl->state) {
	case RL_CALIB_STATE_RESISTANCE:
		done = resistance_phase_run(motor);
		if (done && rl->state == RL_CALIB_STATE_RESISTANCE) {
			/* 电阻阶段成功完成，切换到电感 */
			/* 调试标志：切换到电感测量 */
			motor->data.debug.test_flag1 = 0x20;
			motor->data.debug.test_flag2 = 0;
			rl->state = RL_CALIB_STATE_INDUCTANCE;
			rl->sample_cnt = 0;
			rl->target_samples = RL_IND_SAMPLES;
			rl->last_I_alpha = 0.0f;
			rl->delta_I_sum = 0.0f;
			inverter_enable(&motor->inverter);
			return false; /* 继续执行 */
		}
		return done;

	case RL_CALIB_STATE_INDUCTANCE:
		done = inductance_phase_run(motor);
		return done;

	case RL_CALIB_STATE_FINISH:
		/* 调试标志：RL校准完成 */
		motor->data.debug.test_flag1 = 0x3F;
		motor->data.debug.test_flag2 = 0;
		inverter_disable(&motor->inverter);
		return true;
	case RL_CALIB_STATE_ERROR_RESISTANCE_OUT_OF_RANGE:
		/* 调试标志：电阻错误 */
		motor->data.debug.test_flag1 = 0xEF;
		motor->data.debug.test_flag2 = 1;
		inverter_disable(&motor->inverter);
		return true;
	case RL_CALIB_STATE_ERROR_INDUCTANCE_OUT_OF_RANGE:
		/* 调试标志：电感错误 */
		motor->data.debug.test_flag1 = 0xEF;
		motor->data.debug.test_flag2 = 2;
		inverter_disable(&motor->inverter);
		return true;
	case RL_CALIB_STATE_ERROR_UNBALANCED_PHASES:
		/* 调试标志：相不平衡错误 */
		motor->data.debug.test_flag1 = 0xEF;
		motor->data.debug.test_flag2 = 3;
		inverter_disable(&motor->inverter);
		return true;

	default:
		rl->state = RL_CALIB_STATE_ERROR_RESISTANCE_OUT_OF_RANGE;
		return true;
	}
}

void rl_calib_apply(struct motor *motor)
{
	struct rl_calib *rl;
	struct motor_param_ext *param_ext;

	if (!motor || !motor->param_ext)
		return;

	rl = &motor->calib.rl;
	param_ext = motor->param_ext;

	if (rl->state == RL_CALIB_STATE_FINISH) {
		param_ext->electrical_param.rs = rl->measured_resistance;
		param_ext->electrical_param.ls = rl->measured_inductance;
	}
}
