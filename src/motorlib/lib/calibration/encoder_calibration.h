#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/* 编码器校准对象 - 仅运行时数据 */
struct encoder_calib {
	uint32_t tick_cnt;          /* 滴答计数（10kHz时基） */
	uint8_t state;              /* 子状态机 */
	float elec_angle_acc;       /* 累计电角度（开环旋转累加） */
	uint32_t raw_prev;          /* 上次编码器原始值（解卷绕用） */
	int32_t raw_delta_acc;      /* 编码器累计变化量（解卷绕后） */
	uint32_t align_tick_target; /* 对齐持续时间（ticks） */
	uint32_t align_tick_cnt;    /* 对齐当前计数 */
	float align_angle;          /* 当前对齐目标角度 */
	float align_voltage;        /* 对齐电压 */
	float rotate_voltage;       /* 旋转电压 */
	float rotate_speed;         /* 旋转电角速度（rad/s） */
	uint32_t rotate_tick_target;/* 旋转持续时间（ticks） */
};

/* 编码器校准阶段 */
enum encoder_calib_state {
	ENC_CALIB_IDLE = 0,
	ENC_CALIB_ALIGN,            /* 对齐到0度 */
	ENC_CALIB_ROTATE,           /* 开环旋转 */
	ENC_CALIB_CALC,             /* 计算极对数和方向 */
	ENC_CALIB_OFFSET_ALIGN,     /* 对齐到-90度取偏置 */
	ENC_CALIB_DONE,
	ENC_CALIB_ERROR
};

/* 初始化编码器校准 */
void encoder_calib_init(struct motor *motor);

/* 执行一次编码器校准步进（10kHz调用）
 * @return true-完成, false-继续
 */
bool encoder_calib_run(struct motor *motor);

/* 应用结果到 feedback（极对数、方向、偏置） */
void encoder_calib_apply(struct motor *motor);

#endif
