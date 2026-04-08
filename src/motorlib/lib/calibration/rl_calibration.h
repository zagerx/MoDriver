/**
 * @file rl_calibration.h
 * @brief 电机相电阻/相电感校准模块头文件
 * @details 实现电机相电阻(Rs)和相电感(Ls)的自动测量
 */

#ifndef RL_CALIBRATION_H
#define RL_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/**
 * @brief RL校准状态
 */
enum rl_calib_state {
	RL_CALIB_STATE_IDLE = 0,
	RL_CALIB_STATE_RESISTANCE,
	RL_CALIB_STATE_INDUCTANCE,
	RL_CALIB_STATE_FINISH,
	RL_CALIB_STATE_ERROR_RESISTANCE_OUT_OF_RANGE,
	RL_CALIB_STATE_ERROR_INDUCTANCE_OUT_OF_RANGE,
	RL_CALIB_STATE_ERROR_UNBALANCED_PHASES
};

/**
 * @brief RL校准对象
 */
struct rl_calib {
	enum rl_calib_state state;
	uint32_t sample_cnt;
	uint32_t target_samples;

	/* 运行时数据 - 电阻测量 */
	float current_setpoint; /* 目标电流 [A] */
	float voltage_limit; /* 最大测试电压 [V] */
	float voltage_accumulator; /* 电压积分器 [V] */
	float I_beta_accumulator; /* 用于相平衡检测 */

	/* 运行时数据 - 电感测量 */
	float test_voltage; /* 测试电压 [V] */
	float last_I_alpha; /* 上次I_alpha */
	float delta_I_sum; /* 电流变化累加 */
	bool voltage_polarity; /* 电压极性 */

	/* 测量结果 */
	float measured_resistance; /* [Ohm] */
	float measured_inductance; /* [H] */
};

void rl_calib_init(struct motor *motor);
bool rl_calib_run(struct motor *motor);
void rl_calib_apply(struct motor *motor);

#endif /* RL_CALIBRATION_H */
