
/**
 * @file calibration.c
 * @brief 校准任务主控实现
 * @details 唯一状态机，扁平化管理所有校准阶段
 * 按顺序执行：电流校准 → RL校准 → 编码器校准
 */

#include "calibration.h"
#include "current_calibration.h"
#include "rl_calibration.h"
#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "open_loop.h"

/**
 * @brief 初始化校准模块
 * @param[in] motor 电机实例
 */
void calibration_init(struct motor *motor)
{
	struct calibration *calib;

	if (!motor) {
		return;
	}

	calib = &motor->calib;
	calib->state = CAL_STATE_IDLE;
	calib->error_code = CAL_ERR_NONE;
}

/**
 * @brief 获取校准错误码（调试用）
 * @param[in] motor 电机实例
 * @return 错误码
 */
enum calib_error calibration_get_error(struct motor *motor)
{
	if (!motor) {
		return CAL_ERR_NONE;
	}
	return motor->calib.error_code;
}

/**
 * @brief 校准任务主入口
 * @param[in] motor 电机实例
 * @return true=校准完成（成功或失败）, false=进行中
 * @note 应在主循环中周期性调用
 */
bool calibration_task(struct motor *motor)
{
	struct calibration *calib;
	int ret;
	bool done = false;

	if (!motor) {
		return true;
	}

	calib = &motor->calib;

	switch (calib->state) {
	/* -------- 启动 -------- */
	case CAL_STATE_IDLE:
		curr_calib_prepare(motor);
		calib->state = CAL_STATE_CURRENT_INIT;
		break;

	/* -------- 电流校准 -------- */
	case CAL_STATE_CURRENT_INIT:
		calib->state = CAL_STATE_CURRENT_SAMPLING;
		break;

	case CAL_STATE_CURRENT_SAMPLING:
		ret = curr_calib_step(motor);
		if (ret > 0) {
			curr_calib_apply(motor);
			rl_calib_prepare(motor);
			calib->state = CAL_STATE_RL_INIT;
		}
		break;

	/* -------- RL校准 -------- */
	case CAL_STATE_RL_INIT:
		calib->state = CAL_STATE_RL_RESISTANCE;
		break;

	case CAL_STATE_RL_RESISTANCE:
		ret = rl_resistance_step(motor);
		if (ret < 0) {
			calib->error_code = CAL_ERR_RES_RANGE;
			calib->state = CAL_STATE_FAILED;
		} else if (ret > 0) {
			rl_inductance_prepare(motor);
			calib->state = CAL_STATE_RL_INDUCTANCE;
		}
		break;

	case CAL_STATE_RL_INDUCTANCE:
		ret = rl_inductance_step(motor);
		if (ret < 0) {
			calib->error_code = CAL_ERR_IND_RANGE;
			calib->state = CAL_STATE_FAILED;
		} else if (ret > 0) {
			rl_calib_apply(motor);
			enc_calib_prepare(motor);
			calib->state = CAL_STATE_ENC_ALIGN;
		}
		break;

	/* -------- 编码器校准 -------- */
	case CAL_STATE_ENC_ALIGN:
		ret = enc_align_step(motor);
		if (ret > 0) {
			calib->state = CAL_STATE_ENC_SCAN_FORWARD;
		}
		break;

	case CAL_STATE_ENC_SCAN_FORWARD:
		ret = enc_scan_forward_step(motor);
		if (ret > 0) {
			calib->state = CAL_STATE_ENC_CHECK;
		}
		break;

	case CAL_STATE_ENC_CHECK: {
		bool ok = enc_check_response(motor, &calib->enc.scan_delta);
		if (!ok) {
			calib->error_code = CAL_ERR_ENC_NO_RESPONSE;
			calib->state = CAL_STATE_FAILED;
		} else {
			calib->state = CAL_STATE_ENC_SCAN_BACKWARD;
		}
	} break;

	case CAL_STATE_ENC_SCAN_BACKWARD:
		ret = enc_scan_backward_step(motor);
		if (ret > 0) {
			calib->state = CAL_STATE_ENC_CALC_OFFSET;
		}
		break;

	case CAL_STATE_ENC_CALC_OFFSET:
		enc_calc_offset(motor, calib->enc.scan_delta);
		enc_calib_apply(motor);
		calib->state = CAL_STATE_SUCCESS;
		done = true;
		break;

	/* -------- 结束 -------- */
	case CAL_STATE_SUCCESS:
	case CAL_STATE_FAILED:
		done = true;
		break;

	default:
		calib->state = CAL_STATE_FAILED;
		done = true;
		break;
	}

	return done;
}
