#include "calibration.h"
enum calibration_status calibration_current_task(struct calibration *calib)
{
	switch (calib->status) {
	case CALIBRATION_STATUS_IDLE:
		/* 进入电流校准状态 */
		calib->status = CALIBRATION_STATUS_CURRENT;
		break;
	case CALIBRATION_STATUS_CURRENT:
		break;
	case CALIBRATION_STATUS_SUCCESS:
		break;
	case CALIBRATION_STATUS_FAILED:
		break;
	default:
		break;
	}
	return calib->status;
}
enum calibration_status calibration_task(struct calibration *calib)
{
	switch (calib->status) {
	case CALIBRATION_STATUS_IDLE:
		/* 进入电流校准状态 */
		calib->status = CALIBRATION_STATUS_CURRENT;
		break;
	case CALIBRATION_STATUS_CURRENT:
		break;
	case CALIBRATION_STATUS_ENCODER:
		break;
	case CALIBRATION_STATUS_SUCCESS:
		break;
	case CALIBRATION_STATUS_FAILED:
		break;
	default:
		break;
	}
}
