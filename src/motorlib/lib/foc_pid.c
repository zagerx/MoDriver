// SPDX-License-Identifier: GPL-2.0

#include "foc_pid.h"

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
float foc_pid_run(struct foc_pid *pid, float target, float meas, float dt)
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
 * @brief FOC PID 饱和反馈处理
 * @param pid PID 控制器结构体指针
 * @param output_real 实际输出值
 * @param output_desire 期望输出值
 */
void foc_pid_saturation_feedback(struct foc_pid *pid, float output_real, float output_desire)
{
	// 如果实际输出等于期望输出，说明没饱和，啥都不用做
	if (output_real == output_desire) {
		return;
	}

	float scale = output_real / output_desire;

	pid->integral *= scale;
}

/**
 * @brief 运行 FOC 电流环 PID 控制器（优化版本）
 * @param pid PID 控制器结构体指针
 * @param target 目标值
 * @param meas 测量值
 * @param dt 采样周期 (例如 10kHz 就是 0.0001f)
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

/**
 * @brief FOC 电流 PID 饱和处理
 * @param pid PID 控制器结构体指针
 * @param output_real 实际输出值
 * @param output_desire 期望输出值
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
