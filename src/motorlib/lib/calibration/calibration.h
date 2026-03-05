#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
struct current_calibration {
	uint16_t i_a_offset; /**< @brief a轴电流参考值 */
	uint16_t i_b_offset; /**< @brief b轴电流参考值 */
	uint16_t i_c_offset; /**< @brief c轴电流参考值 */
};
enum calibration_status {
	CALIBRATION_STATUS_IDLE = 0,
	CALIBRATION_STATUS_CURRENT,
	CALIBRATION_STATUS_ENCODER,
	CALIBRATION_STATUS_SUCCESS,
	CALIBRATION_STATUS_FAILED
};
struct calibration {
	enum calibration_status status;
	struct current_calibration current_calib; /**< @brief 电流校准数据 */
};
#endif /* CALIBRATION_H */