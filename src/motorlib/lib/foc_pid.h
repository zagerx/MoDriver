#ifndef __FOC_PID_H
#define __FOC_PID_H

#include <stdint.h>
#include "motor_driver.h"
struct foc_pid {
	struct foc_pid_param *params;

	float err_prev; // 上一次误差 (如果需要D项，电流环通常不需要D)
	float integral; // 积分累加器
};

// 初始化
void foc_pid_init(struct foc_pid *pid, float kp, float ki, float limit);

// 复位
void foc_pid_reset(struct foc_pid *pid);

// 核心计算
float foc_pid_run(struct foc_pid *pid, float target, float meas, float dt);
void foc_pid_saturation_feedback(struct foc_pid *pid, float output_real, float output_desire);

void foc_currentpid_saturation(struct foc_pid *pid, float output_real, float output_desire);
float foc_currentloop_pid_run(struct foc_pid *pid, float target, float meas, float dt);
#endif
