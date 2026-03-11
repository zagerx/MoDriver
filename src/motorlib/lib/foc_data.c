#include "foc_data.h"

/**
 * @brief 绑定FOC数据源
 * @param[in] data FOC数据实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_data_bind(struct foc_data *data, struct feedback_output *feeback,
		   struct currsmp_output *currsmp_out)
{
	if (data) {
		data->meas.feeback = feeback;
		data->meas.currsmp = currsmp_out;
	}
}
