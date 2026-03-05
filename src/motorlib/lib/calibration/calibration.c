#include "calibration.h"
#include "current_calibration.h"
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
		/* TODO: 编码器校准 */
		calib->status = CALIBRATION_STATUS_SUCCESS;
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
