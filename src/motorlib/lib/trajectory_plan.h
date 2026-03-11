#ifndef FOC_TRAJECTORY_PLAN_H
#define FOC_TRAJECTORY_PLAN_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_driver.h"
typedef enum {
	TRAJ_EXEC_IDLE = 0,
	TRAJ_EXEC_RUNNING,
	TRAJ_EXEC_STOPPING, // 新增：紧急停止中
	TRAJ_EXEC_END,
} trajectory_actor_state_t;

typedef struct {
	float t; /* 本段持续时间 (s) */
	float a; /* 加速度 */
} traj_seg_t;

typedef struct {
	/* ===== 执行状态 ===== */
	int cur_seg;    /* 当前段索引 */
	int seg_cnt;    /* 段总数 */
	float seg_time; /* 当前段已运行时间 */

	/* ===== 物理状态（连续量）===== */
	float pos; /* 当前位置 (m) */
	float vel; /* 当前速度 (m/s) */
	float acc; /* 当前加速度 (m/s^2) */

	/* ===== 状态机 ===== */
	trajectory_actor_state_t state; /* IDLE / RUNNING / END */

	/* ===== 轨迹段指针 ===== */
	const traj_seg_t *segs;

} traj_exec_data_t;

typedef enum {
	TRAJ_PLAN_OK = 0,          // 规划成功
	TRAJ_PLAN_NOT_NEEDED,      // 无需规划
	TRAJ_PLAN_ERR_PARAM,       // 参数错误
	TRAJ_PLAN_ERR_NO_SOLUTION, // 轨迹不可达
	TRAJ_PLAN_SEG_TIME_TOO_SHORT,
	TRAJ_PLAN_ERR_BUSY, // 新增：停止过程中不允许操作
} traj_plan_status_t;

typedef struct {
	float p0; // 起始位置
	float v0; // 起始速度
	float a0; // 起始加速度

	float v;         // 当前速度
	float acc;       // 统一的加减速度（绝对值）
	float brake_acc; // 紧急刹停加速度

} traj_plan_input_t;
typedef struct {
	traj_seg_t segs[7]; // 最多7段轨迹
	int seg_cnt;
} traj_plan_output_t;

typedef struct {
	traj_plan_input_t plan_in;
	traj_plan_output_t plan_out;
	float pre_pos; // 上一次的目标位置  仅用来判断是否需要重规划
	float act_pos; //

	/* ===== 紧急停止专用轨迹段===== */
	traj_seg_t emergency_stop_seg; // 用于立即停止的零长度轨迹段

} traj_plan_data_t;

typedef struct {
	traj_plan_data_t plan_data;
	traj_exec_data_t exec_data;
} trajectory_data_t;

struct trajectory_plan {
	trajectory_data_t data; // 指向trajectory_data_t的指针
	trajectory_param_t *param;
};
/* ================= API ================= */
void trajectory_planner_bind_param(struct trajectory_plan *trajectory, trajectory_param_t *param);

int trajectory_planner_init(struct trajectory_plan *trajectory, float start_pos, float start_vel,
			    float start_acc, float exex_cycle);
void trajectory_planner_action(struct trajectory_plan *trajectory, const float dt);
traj_plan_status_t trajectory_planner_update_target(struct trajectory_plan *trajectory,
						    float new_target_pos, float new_vel);

// 新增：紧急停止函数
void trajectory_planner_stop(struct trajectory_plan *trajectory);

float trajectory_planner_read_plantarget(const struct trajectory_plan *trajectory);

float trajectory_planner_get_pos(const struct trajectory_plan *trajectory);
float trajectory_planner_get_vel(const struct trajectory_plan *trajectory);
float trajectory_planner_get_acc(const struct trajectory_plan *trajectory);

#endif /* FOC_TRAJECTORY_PLAN_H */