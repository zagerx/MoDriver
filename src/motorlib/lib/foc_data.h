#ifndef FOC_DATA_H
#define FOC_DATA_H

#include "feedback.h"

struct foc_measurement {
	float vbus;                     /**< @brief 母线电压 V */
	float i_a;                      /**< @brief A相电流 A */
	float i_b;                      /**< @brief B相电流 A */
	float i_c;                      /**< @brief C相电流 A */
	float i_d;                      /**< @brief d轴电流 A */
	float i_q;                      /**< @brief q轴电流 A */
	struct feedback_output *fb_out; /**< @brief 反馈输出数据 */
};

struct foc_data {
	struct foc_measurement meas; /**< @brief FOC测量数据 */
};

void foc_data_bind(struct foc_data *data, struct feedback_output *fb_out);

#endif // FOC_DATA_H