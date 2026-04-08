
/**
 * @file calibration.c
 * @brief 电机校准模块主控实现
 * @details 按顺序调度电流校准和编码器校准两个阶段的状态机
 */

#include "calibration.h"
#include "current_calibration.h"
#include "rl_calibration.h"
#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include <stdint.h>

/**
 * @brief 初始化校准模块
 * @param[in] motor 电机实例
 * @return 无
 * @details 重置校准状态，准备开始校准流程
 */
void calibration_init(struct motor *motor)
{
	struct calibration *calib;

	if (!motor) {
		return;
	}

	calib = &motor->calib;
	calib->status = CALIBRATION_STATUS_IDLE;
	calib->curr_state = CURRENT_STATE_IDLE;
	calib->enc_phase = ENCODER_PHASE_INIT;
}

/**
 * @brief 电流校准阶段处理
 * @param[in] motor 电机实例
 * @return 校准状态
 * @retval 0 校准成功
 * @retval -1 校准失败
 * @retval 1 校准进行中
 */
static int16_t current_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;
	int16_t ret;

	ret = 1;

	switch (calib->curr_state) {
	case CURRENT_STATE_IDLE:
		current_calib_init(motor, 0);
		calib->curr_state = CURRENT_STATE_SAMPLING;
		break;

	case CURRENT_STATE_SAMPLING:
		if (current_calib_run(motor)) {
			current_calib_apply(motor);
			calib->curr_state = CURRENT_STATE_FINISH;
		}
		ret = 1;
		break;

	case CURRENT_STATE_FINISH:
		ret = 0;
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

/**
 * @brief RL校准阶段处理
 * @param[in] motor 电机实例
 * @return 校准状态
 * @retval 0 校准成功
 * @retval -1 校准失败
 * @retval 1 校准进行中
 */
static int16_t rl_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;

	switch (calib->rl.state) {
	case RL_CALIB_STATE_IDLE:
		rl_calib_init(motor);
		return 1;

	case RL_CALIB_STATE_RESISTANCE:
	case RL_CALIB_STATE_INDUCTANCE:
		if (rl_calib_run(motor)) {
			if (calib->rl.state == RL_CALIB_STATE_FINISH) {
				rl_calib_apply(motor);
				return 0;
			} else {
				return -1; /* 错误状态 */
			}
		}
		return 1;

	case RL_CALIB_STATE_FINISH:
		return 0;

	default:
		return -1; /* 错误状态 */
	}
}

/**
 * @brief 编码器校准阶段处理
 * @param[in] motor 电机实例
 * @return 校准状态
 * @retval 0 校准成功
 * @retval -1 校准失败
 * @retval 1 校准进行中
 */
static int16_t encoder_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;
	struct encoder_calib *enc;
	bool done;
	int16_t ret;

	ret = 1;
	enc = &motor->calib.encoder;

	switch (calib->enc_phase) {
	case ENCODER_PHASE_INIT:
		encoder_calib_init(motor);
		calib->enc_phase = ENCODER_PHASE_RUNNING;
		break;

	case ENCODER_PHASE_RUNNING:
		done = encoder_calib_run(motor);
		if (done) {
			/* 检查是否出错 */
			if (enc->state == ENC_CALIB_ERROR_NO_RESPONSE ||
			    enc->state == ENC_CALIB_ERROR_CPR_MISMATCH) {
				ret = -1; /* 错误 */
			} else {
				encoder_calib_apply(motor);
				calib->enc_phase = ENCODER_PHASE_FINISH;
			}
		}
		break;

	case ENCODER_PHASE_FINISH:
		ret = 0;
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

/**
 * @brief 校准任务主入口
 * @param[in] motor 电机实例
 * @return 当前校准状态
 * @note 应在主循环中周期性调用
 * @details 按顺序执行电流校准和编码器校准
 */
enum calibration_status calibration_task(struct motor *motor)
{
	struct calibration *calib;

	if (!motor) {
		return CALIBRATION_STATUS_FAILED;
	}

	calib = &motor->calib;

	switch (calib->status) {
	case CALIBRATION_STATUS_IDLE:
		calib->status = CALIBRATION_STATUS_CURRENT;
		break;

	case CALIBRATION_STATUS_CURRENT: {
		int16_t curr_ret = current_phase_handle(motor);
		if (curr_ret == 0) {
			calib->status = CALIBRATION_STATUS_RL;
		} else if (curr_ret < 0) {
			calib->status = CALIBRATION_STATUS_FAILED;
		}
	} break;

	case CALIBRATION_STATUS_RL: {
		int16_t rl_ret = rl_phase_handle(motor);
		if (rl_ret == 0) {
			calib->status = CALIBRATION_STATUS_ENCODER;
		} else if (rl_ret < 0) {
			calib->status = CALIBRATION_STATUS_FAILED;
		}
	} break;

	case CALIBRATION_STATUS_ENCODER:
		/* 执行编码器校准 */
		{
			int16_t enc_ret = encoder_phase_handle(motor);
			if (enc_ret == 0) {
				calib->status = CALIBRATION_STATUS_SUCCESS;
			} else if (enc_ret < 0) {
				calib->status = CALIBRATION_STATUS_FAILED;
			}
		}
		break;

	case CALIBRATION_STATUS_SUCCESS:
	case CALIBRATION_STATUS_FAILED:
		break;

	default:
		calib->status = CALIBRATION_STATUS_FAILED;
		break;
	}

	return calib->status;
}
