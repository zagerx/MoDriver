

/**
 * @file calibration.h
 * @brief 电机校准模块头文件
 * @details 实现电机参数自动校准功能，包括电流采样校准和编码器校准
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "current_calibration.h"
#include "rl_calibration.h"
#include "encoder_calibration.h"

/** 校准状态枚举 */
enum calibration_status {
	CALIBRATION_STATUS_IDLE = 0, /**< 空闲状态 */
	CALIBRATION_STATUS_CURRENT,  /**< 电流校准阶段 */
	CALIBRATION_STATUS_RL,       /**< RL校准阶段 */
	CALIBRATION_STATUS_ENCODER,  /**< 编码器校准阶段 */
	CALIBRATION_STATUS_SUCCESS,  /**< 校准成功 */
	CALIBRATION_STATUS_FAILED    /**< 校准失败 */
};

/** 编码器校准阶段枚举 */
enum encoder_calib_phase {
	ENCODER_PHASE_INIT = 0, /**< 初始化阶段 */
	ENCODER_PHASE_RUNNING,  /**< 运行阶段 */
	ENCODER_PHASE_FINISH    /**< 完成阶段 */
};

/** 总校准对象（嵌入在 motor 中） */
struct calibration {
	enum calibration_status status;      /**< 当前校准状态 */
	enum current_calib_state curr_state; /**< 电流校准阶段状态 */
	enum encoder_calib_phase enc_phase;  /**< 编码器校准阶段 */

	/** 子校准对象 */
	struct current_calib current; /**< 电流校准数据 */
	struct rl_calib rl;           /**< RL校准数据 */
	struct encoder_calib encoder; /**< 编码器校准数据 */
};

struct motor;

/** 初始化校准模块
 *
 * @param motor 电机对象指针
 */
void calibration_init(struct motor *motor);

/** 校准任务主入口
 *
 * @param motor 电机对象指针
 * @return 校准状态
 */
enum calibration_status calibration_task(struct motor *motor);

#endif /* CALIBRATION_H */
