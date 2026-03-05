#ifndef CURRENT_CALIBRATION_H
#define CURRENT_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/* 电流校准对象（数据结构） */
struct current_calib {
	uint32_t sample_cnt;        /* 采样计数 */
	uint32_t sum_a;             /* a轴累加和 */
	uint32_t sum_b;             /* b轴累加和 */
	uint32_t sum_c;             /* c轴累加和 */
	uint16_t target_samples;    /* 目标采样数 */
};

/* 电流校准阶段 */
enum current_calib_state {
	CURRENT_STATE_IDLE = 0,
	CURRENT_STATE_SAMPLING,
	CURRENT_STATE_FINISH
};

/* 初始化电流校准（禁用逆变器，清零累加器） */
void current_calib_init(struct motor *motor, uint16_t samples);

/* 执行一次采样（非阻塞）
 * @return true-完成, false-继续
 */
bool current_calib_run(struct motor *motor);

/* 应用校准结果到 currsmp */
void current_calib_apply(struct motor *motor);

#endif /* CURRENT_CALIBRATION_H */
