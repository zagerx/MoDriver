#include "foc_pid.h"
// #include <math.h>

// 辅助宏：限制范围
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void foc_pid_init(struct foc_pid *pid, float kp, float ki, float limit)
{
	pid->params->kp = kp;
	pid->params->ki = ki;
	pid->params->limit = limit;
	pid->integral = 0.0f;
	pid->err_prev = 0.0f;
}

void foc_pid_reset(struct foc_pid *pid)
{
	pid->integral = 0.0f;
	pid->err_prev = 0.0f;
}
float foc_pid_run(struct foc_pid *pid, float target, float meas, float dt)
{
	float kp, ki, limit;
	kp = pid->params->kp;
	ki = pid->params->ki;
	limit = pid->params->limit;
	float error = target - meas;

	// 1. P项计算
	float p_term = kp * error;

	// 2. I项计算 (离散化积分: sum += Ki * err * dt)
	// 暂不加到integral里，先预测
	float i_term_predict = pid->integral + (ki * error * dt);

	// 3. 输出预测
	float output = p_term + i_term_predict;

	// 4. 动态抗饱和 (Dynamic Clamping) - ODrive风格
	// 如果输出已经饱和，并且误差试图让输出继续超出饱和区，则停止积分累加
	// 或者：直接限制积分项，使其为了配合P项而不让总输出超限

	// 方案A：简单截断法 (最适合电流环，计算量小)
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
 * @brief 优化后的PID计算
 * @param dt: 采样周期 (例如 10kHz 就是 0.0001f)
 */

float foc_currentloop_pid_run(struct foc_pid *pid, float target, float meas, float dt)
{
	float kp, ki, limit;
	kp = pid->params->kp;
	ki = pid->params->ki;
	limit = pid->params->limit;
	float error = target - meas;

	// 1. P项计算
	float p_term = kp * error;

	// 2. I项计算 (离散化积分: sum += Ki * err * dt)
	// 暂不加到integral里，先预测
	float i_term_predict = pid->integral + (ki * error * dt);

	// 3. 输出预测
	float output = p_term + i_term_predict;

	// 4. 动态抗饱和 (Dynamic Clamping) - ODrive风格
	// 如果输出已经饱和，并且误差试图让输出继续超出饱和区，则停止积分累加
	// 或者：直接限制积分项，使其为了配合P项而不让总输出超限

	// 方案A：简单截断法 (最适合电流环，计算量小)
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

void foc_currentpid_saturation(struct foc_pid *pid, float output_real, float output_desire)
{
	// 如果实际输出等于期望输出，说明没饱和，啥都不用做
	if (output_real == output_desire) {
		return;
	}

	float scale = output_real / output_desire;

	pid->integral *= scale;
}
