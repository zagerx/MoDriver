#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "current_calibration.h"

/* 校准状态 */
enum calibration_status {
	CALIBRATION_STATUS_IDLE = 0,
	CALIBRATION_STATUS_CURRENT,     /* 电流校准阶段 */
	CALIBRATION_STATUS_ENCODER,     /* 编码器校准阶段 */
	CALIBRATION_STATUS_SUCCESS,     /* 校准成功 */
	CALIBRATION_STATUS_FAILED       /* 校准失败 */
};

/* 总校准对象（嵌入在 motor 中） */
struct calibration {
	enum calibration_status status;
	enum current_calib_state curr_state;  /* 电流校准阶段状态 */

	/* 子校准对象 */
	struct current_calib current;         /* 电流校准数据 */
	/* 后续添加: struct encoder_calib encoder; */
};

struct motor;

/* 初始化校准模块 */
void calibration_init(struct motor *motor);

/* 校准任务主入口 */
enum calibration_status calibration_task(struct motor *motor);

#endif /* CALIBRATION_H */
