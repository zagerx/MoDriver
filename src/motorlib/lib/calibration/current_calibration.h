

/**
 * @file current_calibration.h
 * @brief 电流校准模块头文件
 */

#ifndef CURRENT_CALIBRATION_H
#define CURRENT_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/** 电流校准对象（数据结构） */
struct current_calib {
	uint32_t sample_cnt;     /**< 采样计数 */
	uint32_t sum_a;          /**< a轴累加和 */
	uint32_t sum_b;          /**< b轴累加和 */
	uint32_t sum_c;          /**< c轴累加和 */
	uint16_t target_samples; /**< 目标采样数 */
};

/** 电流校准阶段 */
enum current_calib_state {
	CURRENT_STATE_IDLE = 0, /**< 空闲状态 */
	CURRENT_STATE_SAMPLING, /**< 采样中 */
	CURRENT_STATE_FINISH    /**< 校准完成 */
};

/**
 * @brief 初始化电流校准
 * @param[in] motor 电机实例
 * @param[in] samples 目标采样数（0表示使用默认值）
 * @return 无
 * @details 禁用逆变器，清零累加器，准备电流采样
 */
void current_calib_init(struct motor *motor, uint16_t samples);

/**
 * @brief 执行一次电流采样
 * @param[in] motor 电机实例
 * @return true 校准完成，false 需要继续采样
 * @note 应在高频任务中周期性调用
 */
bool current_calib_run(struct motor *motor);

/**
 * @brief 应用电流校准结果
 * @param[in] motor 电机实例
 * @return 无
 * @details 计算平均ADC值并设置为电流采样偏移量
 */
void current_calib_apply(struct motor *motor);

#endif /* CURRENT_CALIBRATION_H */
