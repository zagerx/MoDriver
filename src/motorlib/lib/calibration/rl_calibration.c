
/**
 * @file rl_calibration.c
 * @brief 电机相电阻/相电感校准实现
 * @details 使用电流闭环测量电阻，方波电压测量电感
 * 纯逻辑实现，不维护状态，由 calibration.c 统一状态管理
 */

#include "calibration.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "open_loop.h"
#include "currsmp.h"
#include "motorlib_control_param.h"
#include <math.h>

/* ============ 配置参数 ============ */
#define RL_RES_TEST_CURRENT (1.0f) /* 电阻测量目标电流 [A] */
#define RL_RES_MAX_VOLTAGE  (2.0f) /* 电阻测量最大电压 [V] */
#define RL_RES_KI           (1.0f) /* 电流环积分增益 */
#define RL_RES_SAMPLES      (3000) /* 电阻测量采样次数 */

#define RL_IND_TEST_VOLTAGE (4.0f)  /* 电感测量电压 [V] */
#define RL_IND_SAMPLES      (1250)  /* 电感测量采样次数 */
#define RL_IND_AUDIBLE_FREQ 8000.0f /* 可听方波频率 4kHz */
#define RL_IND_SWITCH_RATIO                                                                        \
	((uint32_t)(ceilf(CONTROL_LOOP_FREQ /                                                      \
			  (2 * RL_IND_AUDIBLE_FREQ)))) /* 极性切换比率，向上取整确保至少为1 */

#define CURRENT_CONTROL_BANDWIDTH (6283.185f) /* 电流环带宽 1000Hz [rad/s] */

#define RL_RES_MIN_VALID       (0.005f)   /* 最小有效电阻 [Ohm] */
#define RL_RES_MAX_VALID       (5.0f)     /* 最大有效电阻 [Ohm] */
#define RL_IND_MIN_VALID       (2e-6f)    /* 最小有效电感 [H] */
#define RL_IND_MAX_VALID       (4000e-6f) /* 最大有效电感 [H] */
#define RL_UNBALANCE_THRESHOLD (0.2f)     /* 相不平衡阈值 */

/* ============ 内部辅助函数 ============ */

/**
 * @brief 获取当前 I_alpha（直接计算）
 */
static inline float get_I_alpha(struct motor *motor)
{
	struct currsmp *currsmp = &motor->currsmp;
	float i_a = (currsmp->input.i_a_raw - currsmp->param->a_chn_offset) * PHASE_CURRENT_GAIN;
	return i_a;
}

/**
 * @brief 获取当前 I_beta（直接计算）
 */
static inline float get_I_beta(struct motor *motor)
{
	struct currsmp *currsmp = &motor->currsmp;
	float i_a = (currsmp->input.i_a_raw - currsmp->param->a_chn_offset) * PHASE_CURRENT_GAIN;
	float i_b = (currsmp->input.i_b_raw - currsmp->param->b_chn_offset) * PHASE_CURRENT_GAIN;
	/* Clarke变换：I_beta = (i_a + 2*i_b) / sqrt(3) */
	return (i_a + 2.0f * i_b) * 0.577350269f;
}

/* ============ 接口实现 ============ */

/**
 * @brief RL校准准备（电阻测量前）
 * @param[in] motor 电机实例
 */
void rl_calib_prepare(struct motor *motor)
{
	struct rl_calib_data *rl;

	if (!motor) {
		return;
	}

	rl = &motor->calib.rl;

	/* 初始化采样计数 */
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

	inverter_enable(&motor->inverter);
}

/**
 * @brief 电感测量准备（电阻完成后调用）
 * @param[in] motor 电机实例
 */
void rl_inductance_prepare(struct motor *motor)
{
	struct rl_calib_data *rl;

	if (!motor) {
		return;
	}

	rl = &motor->calib.rl;

	/* 重置采样计数 */
	rl->sample_cnt = 0;
	rl->target_samples = RL_IND_SAMPLES;

	/* 重置电感测量专用参数 */
	rl->last_I_alpha = 0.0f;
	rl->delta_I_sum = 0.0f;

	/* 调试标志：切换到电感测量 */

	inverter_enable(&motor->inverter);
}

/**
 * @brief 电阻测量单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成, -1=错误
 */
int rl_resistance_step(struct motor *motor)
{
	struct rl_calib_data *rl;
	float I_alpha, I_beta, error;

	if (!motor) {
		return -1;
	}

	rl = &motor->calib.rl;

	/* 获取电流反馈 */
	I_alpha = get_I_alpha(motor);
	I_beta = get_I_beta(motor);

	/* 电流闭环积分 */
	error = rl->current_setpoint - I_alpha;
	rl->voltage_accumulator += RL_RES_KI * CONTROL_PERIOD_DT * error;

	/* 限幅 */
	if (rl->voltage_accumulator > rl->voltage_limit) {
		rl->voltage_accumulator = rl->voltage_limit;
	} else if (rl->voltage_accumulator < -rl->voltage_limit) {
		rl->voltage_accumulator = -rl->voltage_limit;
	}

	/* 累加 I_beta 用于相平衡检测 */
	rl->I_beta_accumulator += 0.01f * CONTROL_PERIOD_DT * (I_beta - rl->I_beta_accumulator);

	/* 施加电压（固定角度0） */
	open_loop_force_align(motor, rl->voltage_accumulator, 0.0f);

	/* 调试标志 */

	rl->sample_cnt++;

	/* 检查完成 */
	if (rl->sample_cnt < rl->target_samples) {
		return 0; /* 继续 */
	}

	/* 阶段完成，计算结果 */
	inverter_disable(&motor->inverter);

	/* 计算电阻 R = V/I */
	if (fabsf(I_alpha) <= 0.1f) {
		/* 电流太小 */
		return -1;
	}

	rl->measured_resistance = fabsf(rl->voltage_accumulator) / fabsf(I_alpha);

	/* 检查相不平衡 */
	if (fabsf(rl->I_beta_accumulator) / rl->current_setpoint > RL_UNBALANCE_THRESHOLD) {
		return -1;
	}

	/* 检查范围 */
	if (rl->measured_resistance < RL_RES_MIN_VALID ||
	    rl->measured_resistance > RL_RES_MAX_VALID) {
		return -1;
	}

	return 1; /* 完成 */
}

/**
 * @brief 电感测量单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成, -1=错误
 */
int rl_inductance_step(struct motor *motor)
{
	struct rl_calib_data *rl;
	float voltage, I_alpha, dI;

	if (!motor) {
		return -1;
	}

	rl = &motor->calib.rl;

	/* 切换电压极性（固定可听频率方波） */
	if (RL_IND_SWITCH_RATIO > 0 && rl->sample_cnt % RL_IND_SWITCH_RATIO == 0) {
		rl->voltage_polarity = !rl->voltage_polarity;
	}
	voltage = rl->voltage_polarity ? rl->test_voltage : -rl->test_voltage;

	/* 施加电压 */
	open_loop_force_align(motor, voltage, 0.0f);

	/* 测量电流 */
	I_alpha = get_I_alpha(motor);

	if (rl->sample_cnt > 0) {
		dI = I_alpha - rl->last_I_alpha;
		rl->delta_I_sum += fabsf(dI);
	}

	rl->last_I_alpha = I_alpha;
	rl->sample_cnt++;

	/* 检查完成 */
	if (rl->sample_cnt < rl->target_samples) {
		return 0; /* 继续 */
	}

	/* 阶段完成，计算结果 */
	inverter_disable(&motor->inverter);

	/* 计算电感 L = V / (dI/dt) */
	float dt = (float)rl->sample_cnt * CONTROL_PERIOD_DT;
	if (rl->delta_I_sum <= 0.1f) {
		/* 电流变化太小 */
		return -1;
	}

	rl->measured_inductance = fabsf(rl->test_voltage) / (rl->delta_I_sum / dt);

	/* 检查范围 */
	if (rl->measured_inductance < RL_IND_MIN_VALID ||
	    rl->measured_inductance > RL_IND_MAX_VALID) {
		return -1;
	}

	return 1; /* 完成 */
}

/**
 * @brief 应用RL校准结果
 * @param[in] motor 电机实例
 */
void rl_calib_apply(struct motor *motor)
{
	struct rl_calib_data *rl;
	struct motor_param_ext *param_ext;
	float kp, ki;

	if (!motor || !motor->param_ext) {
		return;
	}

	rl = &motor->calib.rl;
	param_ext = motor->param_ext;

	param_ext->electrical_param.rs = rl->measured_resistance;
	param_ext->electrical_param.ls = rl->measured_inductance;

	/* 极点配置：带宽 × 电感 = kp，带宽 × 电阻 = ki */
	kp = CURRENT_CONTROL_BANDWIDTH * param_ext->electrical_param.ls;
	ki = CURRENT_CONTROL_BANDWIDTH * param_ext->electrical_param.rs;

	param_ext->foc_param.d_axis.kp = kp;
	param_ext->foc_param.d_axis.ki = ki;
	param_ext->foc_param.q_axis.kp = kp;
	param_ext->foc_param.q_axis.ki = ki;
}
