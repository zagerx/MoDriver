// SPDX-License-Identifier: GPL-2.0

/**
 * @file foc_pid.c
 * @brief FOC PID控制器实现
 * @details 实现位置环、速度环、电流环通用PID计算，包含动态抗饱和处理
 */

#include "foc_pid.h"
#include <stdbool.h>
#include "math.h"
/**
 * @brief 辅助宏：限制数值范围
 * @param x 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限制在[min, max]范围内的值
 */
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 初始化 FOC PID 控制器
 * @param pid PID 控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param limit 输出限幅值
 */
void foc_pid_init(struct foc_pid *pid, float kp, float ki, float limit)
{
	pid->params->kp = kp;
	pid->params->ki = ki;
	pid->params->limit = limit;
	pid->integral = 0.0f;
	pid->err_prev = 0.0f;
}

/**
 * @brief 重置 FOC PID 控制器
 * @param pid PID 控制器结构体指针
 */
void foc_pid_reset(struct foc_pid *pid)
{
	pid->integral = 0.0f;
	pid->err_prev = 0.0f;
}

/**
 * @brief 运行 FOC PID 控制器
 * @param pid PID 控制器结构体指针
 * @param target 目标值
 * @param meas 测量值
 * @param dt 采样周期
 * @return PID 控制器输出值
 */
float foc_pid_positionloop_run(struct foc_pid *pid, float target, float meas, float dt)
{
	float kp, ki, limit;

	kp = pid->params->kp;
	ki = pid->params->ki;
	limit = pid->params->limit;

	float error = target - meas;

	// 1. P 项计算
	float p_term = kp * error;

	// 2. I 项计算 (离散化积分: sum += Ki * err * dt)
	// 暂不加到 integral 里，先预测
	float i_term_predict = pid->integral + (ki * error * dt);

	// 3. 输出预测
	float output = p_term + i_term_predict;

	// 4. 动态抗饱和 (Dynamic Clamping) - ODrive 风格
	// 如果输出已经饱和，并且误差试图让输出继续超出饱和区，则停止积分累加
	// 或者：直接限制积分项，使其为了配合 P 项而不让总输出超限

	// 方案 A：简单截断法 (最适合电流环，计算量小)
	if (output > limit) {
		output = limit;

		// 只有当误差是负的（试图退出饱和区）时，才允许积分变大(反之亦然)
		// 这里采用更激进的方法：反推积分项允许的最大值
		// I_max = Limit - P_term
		float i_max = limit - p_term;

		if (i_term_predict > i_max) {
			i_term_predict = i_max;
		}
	} else if (output < -limit) {
		output = -limit;

		float i_min = -limit - p_term;

		if (i_term_predict < i_min) {
			i_term_predict = i_min;
		}
	}

	// 更新真正的积分器
	pid->integral = i_term_predict;

	return output;
}

/**
 * @brief 电流环PID饱和处理
 * @param[in,out] pid PID控制器实例
 * @param[in] output_real 实际输出值
 * @param[in] output_desire 期望输出值
 */
void foc_currentpid_saturation(struct foc_pid *pid, float output_real, float output_desire)
{
	// 如果实际输出等于期望输出，说明没饱和，啥都不用做
	if (output_real == output_desire) {
		return;
	}

	float scale = output_real / output_desire;

	pid->integral *= scale;
}
/**
 * @brief 运行 FOC PID 控制器
 * @param pid PID 控制器结构体指针
 * @param target 目标值
 * @param meas 测量值
 * @param dt 采样周期
 * @return PID 控制器输出值
 */

float foc_pid_velocityloop_run(struct foc_pid *pid, float target, float meas, float dt)
{
	const float kp = pid->params->kp;
	const float ki = pid->params->ki;
	const float limit = pid->params->limit;
	const float int_limit = 3.0f;
	const float ff = 0.000f;
	const float tau_aw = 1.5f;

	float error = target - meas;

	// 1. 前馈 + P项
	float vel_ff = ff * target;
	float p_term = kp * error;
	float output_pre = p_term + pid->integral + vel_ff;

	// 2. 条件积分（防止继续饱和）
	bool saturate_high = (output_pre > limit);
	bool saturate_low = (output_pre < -limit);
	bool allow_integrate = true;

	if (saturate_high && error > 0) {
		allow_integrate = false;
	}
	if (saturate_low && error < 0) {
		allow_integrate = false;
	}

	if (allow_integrate) {
		pid->integral += ki * error * dt;
	}

	// 3. 时间常数化抗饱和
	if (saturate_high || saturate_low) {
		float desired_integral =
			saturate_high ? (limit - p_term - vel_ff) : (-limit - p_term - vel_ff);

		// 一阶低通向目标值收敛
		float alpha = dt / (tau_aw + dt);
		pid->integral = (1.0f - alpha) * pid->integral + alpha * desired_integral;
	}

	// 4. 限幅与输出
	pid->integral = fmaxf(fminf(pid->integral, int_limit), -int_limit);

	float output = p_term + pid->integral + vel_ff;
	return fmaxf(fminf(output, limit), -limit);
}
/**
 * @brief 运行 FOC 电流环 PID 控制器（优化版本）
 * @param pid PID 控制器结构体指针
 * @param target 目标值
 * @param meas 测量值
 * @param dt 采样周期
 * @return PID 控制器输出值
 */
float foc_currentloop_pid_run(struct foc_pid *pid, float target, float meas, float dt)
{
	float kp, ki, limit;

	kp = pid->params->kp;
	ki = pid->params->ki;
	limit = pid->params->limit;

	float error = target - meas;

	// 1. P 项计算
	float p_term = kp * error;

	// 2. I 项计算 (离散化积分: sum += Ki * err * dt)
	// 暂不加到 integral 里，先预测
	float i_term_predict = pid->integral + (ki * error * dt);

	// 3. 输出预测
	float output = p_term + i_term_predict;

	// 4. 动态抗饱和 (Dynamic Clamping) - ODrive 风格
	// 如果输出已经饱和，并且误差试图让输出继续超出饱和区，则停止积分累加
	// 或者：直接限制积分项，使其为了配合 P 项而不让总输出超限

	// 方案 A：简单截断法 (最适合电流环，计算量小)
	if (output > limit) {
		output = limit;

		// 只有当误差是负的（试图退出饱和区）时，才允许积分变大(反之亦然)
		// 这里采用更激进的方法：反推积分项允许的最大值
		// I_max = Limit - P_term
		float i_max = limit - p_term;

		if (i_term_predict > i_max) {
			i_term_predict = i_max;
		}
	} else if (output < -limit) {
		output = -limit;

		float i_min = -limit - p_term;

		if (i_term_predict < i_min) {
			i_term_predict = i_min;
		}
	}

	// 更新真正的积分器
	pid->integral = i_term_predict;

	return output;
}


