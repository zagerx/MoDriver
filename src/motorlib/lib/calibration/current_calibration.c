
/**
 * @file current_calibration.c
 * @brief 电流校准实现
 * @details 实现电流采样偏移量的自动采集、累加平均及应用
 * 纯逻辑实现，不维护状态，由 calibration.c 统一状态管理
 */

#include "calibration.h"
#include "_motorlib_internal.h"
#include "currsmp.h"
#include "inverter.h"

#define CURRENT_CALIB_DEFAULT_SAMPLES 1000u

/**
 * @brief 电流校准准备
 * @param[in] motor 电机实例
 */
void curr_calib_prepare(struct motor *motor)
{
	struct current_calib_data *curr;

	if (!motor) {
		return;
	}

	curr = &motor->calib.curr;

	curr->sample_cnt = 0;
	curr->sum_a = 0;
	curr->sum_b = 0;
	curr->sum_c = 0;
	curr->target_samples = CURRENT_CALIB_DEFAULT_SAMPLES;

	/* 禁用逆变器 */
	struct inverter *inverter = &motor->inverter;
	if (inverter) {
		inverter_disable(inverter);
	}
}

/**
 * @brief 执行一次电流采样
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 * @note 应在高频任务中周期性调用
 */
int curr_calib_step(struct motor *motor)
{
	struct current_calib_data *curr;
	struct currsmp_input input;
	struct currsmp *currsmp;

	if (!motor) {
		return 1;
	}

	currsmp = &motor->currsmp;
	if (!currsmp) {
		return 1;
	}

	curr = &motor->calib.curr;

	currsmp_get_raw(currsmp, &input);

	curr->sum_a += input.i_a_raw;
	curr->sum_b += input.i_b_raw;
	curr->sum_c += input.i_c_raw;
	curr->sample_cnt++;

	if (curr->sample_cnt >= curr->target_samples) {
		return 1;  /* 完成 */
	}
	return 0;  /* 继续 */
}

/**
 * @brief 应用电流校准结果
 * @param[in] motor 电机实例
 * @details 计算平均ADC值并设置为电流采样偏移量
 */
void curr_calib_apply(struct motor *motor)
{
	struct current_calib_data *curr;
	struct currsmp *currsmp;
	uint16_t offsets[3];

	if (!motor) {
		return;
	}

	currsmp = &motor->currsmp;
	if (!currsmp) {
		return;
	}

	curr = &motor->calib.curr;

	if (curr->sample_cnt == 0) {
		return;
	}

	offsets[0] = (uint16_t)(curr->sum_a / curr->sample_cnt);
	offsets[1] = (uint16_t)(curr->sum_b / curr->sample_cnt);
	offsets[2] = (uint16_t)(curr->sum_c / curr->sample_cnt);

	currsmp_update_offset(currsmp, offsets);
}
