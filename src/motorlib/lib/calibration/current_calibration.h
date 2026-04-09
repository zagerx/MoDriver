
/**
 * @file current_calibration.h
 * @brief 电流校准模块头文件
 * @details 仅包含函数声明，供 calibration.c 或其他模块使用
 */

#ifndef CURRENT_CALIBRATION_H
#define CURRENT_CALIBRATION_H

#include <stdint.h>

struct motor;

/**
 * @brief 电流校准准备
 * @param[in] motor 电机实例
 */
void curr_calib_prepare(struct motor *motor);

/**
 * @brief 电流校准单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int curr_calib_step(struct motor *motor);

/**
 * @brief 应用电流校准结果
 * @param[in] motor 电机实例
 */
void curr_calib_apply(struct motor *motor);

#endif /* CURRENT_CALIBRATION_H */
