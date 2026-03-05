#include "calibration.h"
#include "current_calibration.h"
#include "encoder_calibration.h"
#include "_motorlib_internal.h"
#include "inverter.h"

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
static enum calibration_status current_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;
	bool done;

	switch (calib->curr_state) {
	case CURRENT_STATE_IDLE:
		current_calib_init(motor, 0);
		calib->curr_state = CURRENT_STATE_SAMPLING;
		break;

	case CURRENT_STATE_SAMPLING:
		done = current_calib_run(motor);
		if (done) {
			calib->curr_state = CURRENT_STATE_FINISH;
		}
		break;

	case CURRENT_STATE_FINISH:
		current_calib_apply(motor);
		calib->curr_state = CURRENT_STATE_IDLE; /* 重置 */
		return CALIBRATION_STATUS_ENCODER;

	default:
		return CALIBRATION_STATUS_FAILED;
	}

	return CALIBRATION_STATUS_CURRENT;
}

/* 编码器校准阶段处理 */
static enum calibration_status encoder_phase_handle(struct motor *motor)
{
	struct calibration *calib = &motor->calib;
	bool done;

	switch (calib->enc_phase) {
	case ENCODER_PHASE_INIT:
		encoder_calib_init(motor);
		calib->enc_phase = ENCODER_PHASE_RUNNING;
		break;

	case ENCODER_PHASE_RUNNING:
		done = encoder_calib_run(motor);
		if (done) {
			calib->enc_phase = ENCODER_PHASE_FINISH;
		}
		break;

	case ENCODER_PHASE_FINISH:
		encoder_calib_apply(motor);
		calib->enc_phase = ENCODER_PHASE_INIT; /* 重置 */
		return CALIBRATION_STATUS_SUCCESS;

	default:
		return CALIBRATION_STATUS_FAILED;
	}

	return CALIBRATION_STATUS_ENCODER;
}

enum calibration_status calibration_task(struct motor *motor)
{
	struct calibration *calib;

	if (!motor) {
		return CALIBRATION_STATUS_FAILED;
	}

	calib = &motor->calib;

	switch (calib->status) {
	case CALIBRATION_STATUS_IDLE:
		calibration_init(motor);
		calib->status = CALIBRATION_STATUS_CURRENT;
		break;

	case CALIBRATION_STATUS_CURRENT:
		calib->status = current_phase_handle(motor);
		break;

	case CALIBRATION_STATUS_ENCODER:
		/* 执行编码器校准 */
		calib->status = encoder_phase_handle(motor);
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
