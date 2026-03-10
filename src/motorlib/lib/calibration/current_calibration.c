#include "current_calibration.h"
#include "_motorlib_internal.h"
#include "calibration.h"
#include "currsmp.h"
#include "inverter.h"

#define CURRENT_CALIB_DEFAULT_SAMPLES 1000u

/**
 * @brief 初始化电流校准
 * @param[in] motor 电机实例
 * @param[in] samples 目标采样数（0表示使用默认值）
 * @return 无
 * @details 禁用逆变器，清零累加器，准备电流采样
 */
void current_calib_init(struct motor *motor, uint16_t samples)
{
	struct current_calib *curr;

	if (!motor) {
		return;
	}

	/* 通过 motor 获取电流校准对象 */
	curr = &motor->calib.current;

	curr->sample_cnt = 0;
	curr->sum_a = 0;
	curr->sum_b = 0;
	curr->sum_c = 0;
	curr->target_samples = (samples > 0) ? samples : CURRENT_CALIB_DEFAULT_SAMPLES;

	/* 禁用逆变器 */
	if (motor->inverter) {
		inverter_disable(motor->inverter);
	}
}

/**
 * @brief 执行一次电流采样
 * @param[in] motor 电机实例
 * @return true 校准完成，false 需要继续采样
 * @note 应在高频任务中周期性调用
 */
bool current_calib_run(struct motor *motor)
{
	struct current_calib *curr;
	struct currsmp_input input;

	if (!motor || !motor->currsmp) {
		return true;
	}

	curr = &motor->calib.current;

	currsmp_get_raw(motor->currsmp, &input);
	curr->sum_a += input.i_a_raw;
	curr->sum_b += input.i_b_raw;
	curr->sum_c += input.i_c_raw;
	curr->sample_cnt++;

	return (curr->sample_cnt >= curr->target_samples);
}

/**
 * @brief 应用电流校准结果
 * @param[in] motor 电机实例
 * @return 无
 * @details 计算平均ADC值并设置为电流采样偏移量
 */
void current_calib_apply(struct motor *motor)
{
	struct current_calib *curr;
	struct currsmp *currsmp;
	uint16_t offsets[3];

	if (!motor || !motor->currsmp || !motor->currsmp->param) {
		return;
	}

	curr = &motor->calib.current;
	currsmp = motor->currsmp;

	if (curr->sample_cnt == 0) {
		return;
	}

	offsets[0] = (uint16_t)(curr->sum_a / curr->sample_cnt);
	offsets[1] = (uint16_t)(curr->sum_b / curr->sample_cnt);
	offsets[2] = (uint16_t)(curr->sum_c / curr->sample_cnt);

	currsmp_update_offset(currsmp, offsets);
}
