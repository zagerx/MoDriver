#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/* 编码器校准对象 - 仅运行时数据 */
struct encoder_calib {
	uint32_t tick_cnt;       /* 滴答计数（10kHz时基） */
	uint8_t state;           /* 子状态机 */
	uint16_t raw_prev;       /* 上次编码器原始值（解卷绕用） */
	int32_t raw_delta_acc;   /* 编码器累计变化量（解卷绕后） */
	uint32_t align_tick_cnt; /* 对齐当前计数 */
};

/* 编码器校准阶段 */
enum encoder_calib_state {
	ENC_CALIB_IDLE = 0,
	ENC_CALIB_ALIGN,        /* 对齐到0度 */
	ENC_CALIB_ROTATE,       /* 开环旋转 */
	ENC_CALIB_CALC,         /* 计算极对数和方向 */
	ENC_CALIB_OFFSET_ALIGN, /* 对齐到-90度取偏置 */
	ENC_CALIB_DONE,
	ENC_CALIB_ERROR
};

/**
 * @brief 初始化编码器校准
 * @param[in] motor 电机实例
 * @return 无
 * @details 重置校准状态，禁用逆变器，准备开始校准
 */
void encoder_calib_init(struct motor *motor);

/**
 * @brief 执行一次编码器校准步进
 * @param[in] motor 电机实例
 * @return true 校准完成，false 需要继续执行
 * @note 应在10kHz高频任务中周期性调用
 */
bool encoder_calib_run(struct motor *motor);

/**
 * @brief 应用编码器校准结果
 * @param[in] motor 电机实例
 * @return 无
 * @note 结果已在校准过程中实时应用，此函数保留用于后续扩展
 */
void encoder_calib_apply(struct motor *motor);

#endif
