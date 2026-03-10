#include "calibration.h"
#include "current_calibration.h"
#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "inverter.h"
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

/* 电流校准阶段处理 */
/**
0:校准成功
-1:校准失败
1:校准进行中
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

/* 编码器校准阶段处理 */
static int16_t encoder_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;
	bool done;
	int16_t ret;
	ret = 1;
	switch (calib->enc_phase) {
	case ENCODER_PHASE_INIT:
		encoder_calib_init(motor);
		calib->enc_phase = ENCODER_PHASE_RUNNING;
		break;

	case ENCODER_PHASE_RUNNING:
		done = encoder_calib_run(motor);
		if (done) {
			encoder_calib_apply(motor);
			calib->enc_phase = ENCODER_PHASE_FINISH;
		}
		ret = 1;
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

	case CALIBRATION_STATUS_CURRENT:
		if (!current_phase_handle(motor)) {
			calib->status = CALIBRATION_STATUS_ENCODER;
		}
		break;

	case CALIBRATION_STATUS_ENCODER:
		/* 执行编码器校准 */
		if (!encoder_phase_handle(motor)) {
			calib->status = CALIBRATION_STATUS_SUCCESS;
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
