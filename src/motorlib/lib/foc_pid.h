

/**
 * @file foc_pid.h
 * @brief FOC PID控制器头文件
 *
 * 该文件定义了FOC（Field-Oriented Control）使用的PID控制器结构和相关接口。
 */

#ifndef __FOC_PID_H
#define __FOC_PID_H

#include <stdint.h>
#include "motor_interface_params.h"

/**
 * @brief FOC PID控制器结构体
 */
struct foc_pid {
	/** PID参数指针 */
	struct foc_pid_param *params;

	/** 上一次误差 (如果需要D项，电流环通常不需要D) */
	float err_prev;

	/** 积分累加器 */
	float integral;
};

/**
 * @brief 初始化PID控制器
 *
 * @param[in,out] pid   PID控制器实例
 * @param[in]     kp    比例增益
 * @param[in]     ki    积分增益
 * @param[in]     limit 输出限制
 */
void foc_pid_init(struct foc_pid *pid, float kp, float ki, float limit);

/**
 * @brief 复位PID控制器
 *
 * @param[in,out] pid PID控制器实例
 */
void foc_pid_reset(struct foc_pid *pid);

/**
 * @brief PID核心计算
 *
 * @param[in,out] pid    PID控制器实例
 * @param[in]     target 目标值
 * @param[in]     meas   测量值
 * @param[in]     dt     时间步长
 *
 * @return 计算后的输出值
 */
float foc_pid_run(struct foc_pid *pid, float target, float meas, float dt);

/**
 * @brief 饱和反馈处理
 *
 * @param[in,out] pid           PID控制器实例
 * @param[in]     output_real   实际输出值
 * @param[in]     output_desire 期望输出值
 */
void foc_pid_saturation_feedback(struct foc_pid *pid, float output_real, float output_desire);

/**
 * @brief 电流环PID饱和处理
 *
 * @param[in,out] pid           PID控制器实例
 * @param[in]     output_real   实际输出值
 * @param[in]     output_desire 期望输出值
 */
void foc_currentpid_saturation(struct foc_pid *pid, float output_real, float output_desire);

/**
 * @brief 电流环PID运行
 *
 * @param[in,out] pid    PID控制器实例
 * @param[in]     target 目标值
 * @param[in]     meas   测量值
 * @param[in]     dt     时间步长
 *
 * @return 计算后的输出值
 */
float foc_currentloop_pid_run(struct foc_pid *pid, float target, float meas, float dt);

#endif /* __FOC_PID_H */
