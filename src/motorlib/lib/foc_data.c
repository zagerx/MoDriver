#include "foc_data.h"

/**
 * @brief 绑定FOC数据源
 * @param[in] data FOC数据实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_data_bind(struct foc_data *data, struct currsmp_output *currsmp_out,
		   struct foc_pid_param *d_axis_pid_param, struct foc_pid_param *q_axis_pid_param,
		   struct foc_pid_param *vel_pid_param, struct foc_pid_param *pos_pid_param)
{
	if (data) {
		data->meas.currsmp = currsmp_out;
		data->ctrl.d_axis.params = d_axis_pid_param;
		data->ctrl.q_axis.params = q_axis_pid_param;
		data->ctrl.velocity.params = vel_pid_param;
		data->ctrl.position.params = pos_pid_param;
	}
}
