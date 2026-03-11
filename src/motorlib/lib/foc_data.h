#ifndef FOC_DATA_H
#define FOC_DATA_H

#include "feedback.h"
#include "currsmp.h"
struct foc_measurement {
	struct currsmp_output *currsmp;  /**< @brief 电流采样输出数据 */
	struct feedback_output *feeback; /**< @brief 反馈输出数据 */
	float i_alpha;                   /**< @brief α轴电流 */
	float i_beta;                    /**< @brief β轴电流 */
	float i_d;                       /**< @brief d轴电流 */
	float i_q;                       /**< @brief q轴电流 */
};
struct foc_reference {
	float i_d;      /**< @brief d轴电流环输入 */
	float i_q;      /**< @brief q轴电流环输入 */
	float velocity; /**< @brief 速度环输入 rad/s */
};
struct foc_data {
	struct foc_measurement meas; /**< @brief FOC测量数据 */
	struct foc_reference ref;    /**< @brief FOC参考输入 */
	float self_eangle;           /**< @brief 电机开环强托自增角度 */
};

/**
 * @brief 绑定FOC数据源
 * @param[in] data FOC数据实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_data_bind(struct foc_data *data, struct feedback_output *feeback,
		   struct currsmp_output *currsmp_out);

#endif // FOC_DATA_H