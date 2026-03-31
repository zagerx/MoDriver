/* SPDX-License-Identifier: GPL-2.0 */

#include "trajectory_plan.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/**
 * @brief 绑定轨迹规划器参数
 *
 * @param trajectory 轨迹规划器实例指针
 * @param param      轨迹参数指针
 *
 * @return 无
 */
void trajectory_planner_bind_param(struct trajectory_plan *trajectory, trajectory_param_t *param)
{
	if (trajectory) {
		trajectory->param = param;
	}
}

/**
 * @brief 初始化轨迹规划器
 *
 * @param trajectory 轨迹规划器实例指针
 * @param start_pos  起始位置
 * @param start_vel  起始速度
 * @param start_acc  起始加速度
 * @param exec_cycle 执行周期
 *
 * @return 0 表示成功
 */
int trajectory_planner_init(struct trajectory_plan *trajectory, float start_pos, float start_vel,
			    float start_acc, float exec_cycle)
{
	(void)exec_cycle; // 当前实现未使用执行周期参数
	(void)start_acc;  // 当前实现未使用起始加速度参数
	(void)start_vel;  // 当前实现未使用起始速度参数
	(void)start_pos;  // 当前实现未使用起始位置参数
	(void)trajectory; // 避免未使用警告
	return 0;
}

void trajectory_planner_action(struct trajectory_plan *trajectory, float dt)
{
	(void)trajectory; // 避免未使用警告
	(void)dt;         // 避免未使用警告
}

void trajectory_planner_stop(struct trajectory_plan *trajectory)
{
	(void)trajectory; // 避免未使用警告
}

traj_plan_status_t trajectory_planner_update_target(struct trajectory_plan *trajectory,
						    float new_target_pos, float new_vel)
{
	trajectory_data_t *d = &trajectory->data;
	traj_exec_data_t *exec = &d->exec_data;

	exec->pos = new_target_pos;
	exec->vel = new_vel;
	return TRAJ_PLAN_OK;
}

/**
 * @brief 获取当前位置
 *
 * @param trajectory   轨迹规划器实例指针
 *
 * @return 当前位置
 */
float trajectory_planner_get_pos(const struct trajectory_plan *trajectory)
{
	const trajectory_data_t *data = &trajectory->data;
	return data->exec_data.pos;
}
/**
 * @brief 获取当前速度
 *
 * @param trajectory   轨迹规划器实例指针
 *
 * @return 当前速度
 */
float trajectory_planner_get_vel(const struct trajectory_plan *trajectory)
{
	const trajectory_data_t *data = &trajectory->data;
	return data->exec_data.vel;
}

/**
 * @brief 获取当前加速度
 *
 * @param trajectory   轨迹规划器实例指针
 *
 * @return 当前加速度
 */
float trajectory_planner_get_acc(const struct trajectory_plan *trajectory)
{
	const trajectory_data_t *data = &trajectory->data;
	return data->exec_data.acc;
}

/**
 * @brief 读取规划目标位置
 *
 * @param trajectory   轨迹规划器实例指针
 *
 * @return 规划目标位置
 */
float trajectory_planner_read_plantarget(const struct trajectory_plan *trajectory)
{
	const trajectory_data_t *data = &trajectory->data;
	return data->plan_data.act_pos;
}
