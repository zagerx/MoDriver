
/**
 * @file rl_calibration.h
 * @brief 电机相电阻/相电感校准模块头文件
 * @details 仅包含函数声明，供 calibration.c 或其他模块使用
 */

#ifndef RL_CALIBRATION_H
#define RL_CALIBRATION_H

#include <stdint.h>

struct motor;

/**
 * @brief RL校准准备（电阻测量前）
 * @param[in] motor 电机实例
 */
void rl_calib_prepare(struct motor *motor);

/**
 * @brief 电感测量准备（电阻完成后调用）
 * @param[in] motor 电机实例
 */
void rl_inductance_prepare(struct motor *motor);

/**
 * @brief 电阻测量单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成, -1=错误
 */
int rl_resistance_step(struct motor *motor);

/**
 * @brief 电感测量单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成, -1=错误
 */
int rl_inductance_step(struct motor *motor);

/**
 * @brief 应用RL校准结果
 * @param[in] motor 电机实例
 */
void rl_calib_apply(struct motor *motor);

#endif /* RL_CALIBRATION_H */
