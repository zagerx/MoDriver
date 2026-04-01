
/**
 * @file trajectory_plan.c
 * @brief 轨迹规划器实现
 * @details 实现三段式梯形速度规划、在线重规划、紧急停止及轨迹执行步进
 */

#include "trajectory_plan.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* =========================================================
 * 内部函数声明
 * ========================================================= */
static void traj_exec_init(traj_exec_data_t *d, float start_pos, float start_vel, float start_acc);
static void traj_exec_step(traj_exec_data_t *d, float dt);
static int traj_exec_load(traj_exec_data_t *d, const traj_seg_t *segs, int seg_cnt);
static traj_plan_status_t traj_plan(traj_plan_data_t *d, float target_pos, float target_vel);
static void traj_plan_init(traj_plan_data_t *d, float start_pos, float start_v, float acc_max,
			   float exec_cycle);
static traj_plan_status_t traj_plan_three_segment(const traj_plan_input_t *in,
						  traj_plan_output_t *out, float target_pos,
						  float target_vel);
/** @brief 纯减速轨迹规划（紧急停止专用） */
static traj_plan_status_t traj_plan_deceleration_only(const traj_plan_input_t *in,
						      traj_plan_output_t *out, float brake_acc);

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
	trajectory_data_t *data = &trajectory->data;
	traj_plan_data_t *plan = &data->plan_data;
	traj_exec_data_t *exec = &data->exec_data;

	traj_exec_init(exec, start_pos, start_vel, start_acc);

	trajectory_param_t *conf = trajectory->param;

	traj_plan_init(plan, start_pos, start_vel, conf->acc_max, exec_cycle);

	return 0;
}

/**
 * @brief 轨迹规划器执行（周期调用）
 *
 * @param trajectory 轨迹规划器实例指针
 * @param dt         时间步长
 *
 * @return 无
 *
 * @note 边界条件：
 *       - END状态：自动转换为IDLE
 *       - RUNNING/STOPPING状态：执行轨迹段
 *       - IDLE状态：无操作
 */
void trajectory_planner_action(struct trajectory_plan *trajectory, float dt)
{
	traj_exec_data_t *exec = &((trajectory_data_t *)&trajectory->data)->exec_data;

	/* END状态：自动清理转换为IDLE */
	if (exec->state == TRAJ_EXEC_END) {
		exec->state = TRAJ_EXEC_IDLE;
		exec->segs = NULL; // 清空指针，防止误用
		exec->seg_cnt = 0;
		exec->cur_seg = 0;
		return;
	}

	/* 执行态：RUNNING或STOPPING都执行轨迹 */
	if (exec->state == TRAJ_EXEC_RUNNING || exec->state == TRAJ_EXEC_STOPPING) {
		/* 检查轨迹数据有效性 */
		if (!exec->segs || exec->seg_cnt <= 0 || exec->cur_seg < 0) {
			/* 数据无效，强制结束 */
			exec->state = TRAJ_EXEC_END;
			exec->acc = 0.0f;
			return;
		}
		traj_exec_step(exec, dt);
	}

	/* IDLE状态：无操作，等待新目标 */
}

/**
 * @brief 紧急停止函数
 *
 * @param trajectory 轨迹规划器实例指针
 *
 * @return 无
 *
 * @note 边界条件：
 *       - 任何非END/IDLE状态都可进入STOPPING
 *       - 立即强制设置STOPPING状态
 *       - 必须保证加载有效轨迹（规划失败时使用零轨迹）
 *       - STOPPING状态重复调用：重新规划（更安全）
 */
void trajectory_planner_stop(struct trajectory_plan *trajectory)
{
	trajectory_data_t *d = &trajectory->data;
	traj_plan_data_t *plan = &d->plan_data;
	traj_exec_data_t *exec = &d->exec_data;

	/* 边界条件检查：只允许从 RUNNING 状态进入 STOPPING */
	if (exec->state != TRAJ_EXEC_RUNNING) {
		return; // 已经在停止、空闲或结束状态，不响应
	}

	/* 立即强制进入STOPPING状态 */
	exec->state = TRAJ_EXEC_STOPPING;
	plan->pre_pos = exec->pos; // 更新为当前位置
	plan->act_pos = exec->pos; // 期望停止在当前位置

	/* 原子化读取当前状态（假设这是原子操作） */
	float current_pos = exec->pos;
	float current_vel = exec->vel;
	float current_acc = exec->acc;

	/* 设置规划输入参数 */
	plan->plan_in.p0 = current_pos;
	plan->plan_in.v0 = current_vel;
	plan->plan_in.a0 = current_acc;
	float brake_acc = plan->plan_in.brake_acc;

	/* 规划纯减速轨迹（速度降到0），使用紧急刹停加速度 */
	traj_plan_status_t ret =
		traj_plan_deceleration_only(&plan->plan_in, &plan->plan_out, brake_acc);

	if (ret == TRAJ_PLAN_OK) {
		/* 规划成功，加载停止轨迹 */
		if (traj_exec_load(exec, plan->plan_out.segs, plan->plan_out.seg_cnt) == 0) {
			/* 成功加载，保持STOPPING状态，执行器将执行减速 */
			return;
		}
		/* 加载失败，执行fallback */
	}

	/* 任何失败情况：使用预置的紧急停止段（零时间，立即结束） */
	plan->emergency_stop_seg.t = 0.0f;
	plan->emergency_stop_seg.a = 0.0f;
	traj_exec_load(exec, &plan->emergency_stop_seg, 1);
}

/**
 * @brief 纯减速轨迹规划（紧急停止专用）
 *
 * @param in         规划输入参数
 * @param out        规划输出结果
 * @param brake_acc  紧急刹停加速度
 *
 * @return 规划状态
 *
 * @note 从当前速度使用紧急减速度减速到0
 */
static traj_plan_status_t traj_plan_deceleration_only(const traj_plan_input_t *in,
						      traj_plan_output_t *out, float brake_acc)
{
	out->seg_cnt = 0;

	float v0 = in->v0;
	float acc = brake_acc; // 使用紧急刹停加速度

	/* 如果速度已经为0，无需规划 */
	if (fabsf(v0) < 1e-6f) {
		return TRAJ_PLAN_NOT_NEEDED;
	}

	/* 确定减速度方向（与速度方向相反） */
	float dir = (v0 > 0.0f) ? -1.0f : 1.0f;

	/* 计算减速时间: t = |v0| / acc */
	float t_dec = fabsf(v0) / acc;

	if (t_dec < 1e-6f) {
		return TRAJ_PLAN_NOT_NEEDED;
	}

	/* 单段减速轨迹 */
	out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_dec, .a = dir * acc};

	return TRAJ_PLAN_OK;
}

/**
 * @brief 更新目标位置（在线 replanning）
 *
 * @param trajectory      轨迹规划器实例指针
 * @param new_target_pos  新的目标位置
 * @param new_vel         新的目标速度
 *
 * @return 规划状态
 *
 * @note 边界条件：
 *       - STOPPING状态：拒绝更新（返回BUSY）
 *       - IDLE/RUNNING/END状态：允许更新
 *       - 加载成功后强制设为RUNNING
 */
traj_plan_status_t trajectory_planner_update_target(struct trajectory_plan *trajectory,
						    float new_target_pos, float new_vel)
{
	trajectory_data_t *d = &trajectory->data;
	traj_plan_data_t *plan = &d->plan_data;
	traj_exec_data_t *exec = &d->exec_data;
	trajectory_param_t *cfg = trajectory->param;

	/* 只允许从 IDLE 或 END 状态开始新轨迹 */
	/* RUNNING 状态可以重规划（在线更新） */
	/* STOPPING 状态拒绝任何更新 */
	if (exec->state == TRAJ_EXEC_STOPPING) {
		return TRAJ_PLAN_ERR_BUSY;
	}

	/* 检查目标位置是否变化 */
	if (fabsf(new_target_pos - plan->pre_pos) < 1e-4f) {
		return TRAJ_PLAN_NOT_NEEDED;
	}

	/* 限制目标速度 */
	if (fabsf(new_vel) > cfg->vmax) {
		new_vel = cfg->vmax;
	}

	/* 设置规划输入 */
	plan->plan_in.p0 = exec->pos;
	plan->plan_in.v0 = exec->vel;
	plan->plan_in.a0 = exec->acc;

	traj_plan_status_t ret = traj_plan(plan, new_target_pos, new_vel);

	if (ret != TRAJ_PLAN_OK) {
		return ret;
	}

	/* 加载新轨迹 */
	if (traj_exec_load(exec, plan->plan_out.segs, plan->plan_out.seg_cnt) != 0) {
		return TRAJ_PLAN_ERR_PARAM;
	}

	/* 加载成功，强制设为RUNNING（无论之前是什么状态） */
	exec->state = TRAJ_EXEC_RUNNING;

	plan->pre_pos = new_target_pos;
	plan->act_pos = new_target_pos;

	return TRAJ_PLAN_OK;
}

/**
 * @brief 核心规划入口
 *
 * @param d            规划数据
 * @param target_pos   目标位置
 * @param target_vel   目标速度
 *
 * @return 规划状态
 */
static traj_plan_status_t traj_plan(traj_plan_data_t *d, float target_pos, float target_vel)
{
	traj_plan_input_t *in = &d->plan_in;
	traj_plan_output_t *out = &d->plan_out;

	/* 参数检查 */
	if (!d || in->acc <= 0.0f || in->brake_acc <= 0.0f) {
		return TRAJ_PLAN_ERR_PARAM;
	}

	/* 加速度检查 */
	if (fabsf(in->a0) > in->brake_acc * 1.1f) // 检查是否超过紧急加速度
	{
		return TRAJ_PLAN_ERR_PARAM;
	}

	return traj_plan_three_segment(in, out, target_pos, target_vel);
}

/**
 * @brief 三段式（梯形）速度规划
 *
 * @param in           规划输入参数
 * @param out          规划输出结果
 * @param target_pos   目标位置
 * @param target_vel   目标速度
 *
 * @return 规划状态
 *
 * @note 根据运动方向智能选择加速度：
 *       - 正常加减速使用 acc
 *       - 方向相反的刹停使用 brake_acc
 *       同向距离不足时，减速后自动反向修正至目标点
 */
#define TRAJ_POS_EPSILON  1e-4f
#define TRAJ_VEL_EPSILON  1e-6f
#define TRAJ_TIME_EPSILON 1e-7f

static traj_plan_status_t traj_plan_three_segment(const traj_plan_input_t *in,
						  traj_plan_output_t *out, float target_pos,
						  float target_vel)
{
	out->seg_cnt = 0;

	float p0 = in->p0;
	float v0 = in->v0;
	float dp = target_pos - p0;

	float acc = in->acc;             // 正常加减速度
	float brake_acc = in->brake_acc; // 紧急刹停加速度
	float v_max = target_vel;        // 目标巡航速度（峰值上限）

	/* 情况1：当前速度与目标方向相反 → 使用紧急加速度刹停 */
	if ((dp > TRAJ_POS_EPSILON && v0 < -TRAJ_VEL_EPSILON) ||
	    (dp < -TRAJ_POS_EPSILON && v0 > TRAJ_VEL_EPSILON)) {
		/* 使用紧急加速度刹停 */
		float t_brake = fabsf(v0) / brake_acc;
		float a_brake = (v0 > 0.0f) ? -brake_acc : brake_acc;

		out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_brake, .a = a_brake};

		/* 计算刹停位移并更新状态 */
		float brake_disp = v0 * t_brake + 0.5f * a_brake * t_brake * t_brake;
		p0 = p0 + brake_disp;
		v0 = 0.0f;
		dp = target_pos - p0;
	}

	/* 使用绝对值进行后续计算 */
	float dp_norm = fabsf(dp);
	float v0_norm = fabsf(v0);
	float move_dir = (dp >= 0.0f) ? 1.0f : -1.0f; // 最终运动方向

	/* ---------- 情况2：距离不足以用正常减速度在目标点停准 ---------- */
	float d_stop = (v0_norm * v0_norm) / (2.0f * acc);
	if (dp_norm <= d_stop + TRAJ_POS_EPSILON) {
		/* 2.1 如果当前有速度，先以正常减速度减速到0（必定超调） */
		if (v0_norm > TRAJ_VEL_EPSILON) {
			float t_dec = v0_norm / acc;
			float a_dec = -move_dir * acc; // 减速方向与运动方向相反

			out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_dec, .a = a_dec};

			/* 更新减速结束后的位置和速度 */
			p0 = p0 + v0 * t_dec + 0.5f * a_dec * t_dec * t_dec;
			v0 = 0.0f;
			dp = target_pos - p0;
			dp_norm = fabsf(dp);
			move_dir = (dp >= 0.0f) ? 1.0f : -1.0f;
			v0_norm = 0.0f;
		}

		/* 2.2 减速后可能已超调（dp为负）或仍未到达，需从静止反向归位 */
		if (dp_norm > TRAJ_POS_EPSILON) {
			/* 从静止运动距离 dp_norm，使用正常加速度 acc */
			float v_peak = sqrtf(dp_norm * acc); // 加速-减速所需峰值速度
			if (v_peak > v_max) {
				v_peak = v_max;
			}

			if (v_peak > TRAJ_VEL_EPSILON) {
				/* 加速段 */
				float t_acc = v_peak / acc;
				out->segs[out->seg_cnt++] =
					(traj_seg_t){.t = t_acc, .a = move_dir * acc};

				/* 减速段 */
				float t_dec = v_peak / acc;
				out->segs[out->seg_cnt++] =
					(traj_seg_t){.t = t_dec, .a = -move_dir * acc};
			}
		}
		return TRAJ_PLAN_OK;
	}

	/* ---------- 情况3：距离足够，可采用梯形或三角形速度曲线 ---------- */
	/* 计算加速/减速到 v_max 所需距离和时间 */
	float d_acc = 0.0f, t_acc = 0.0f;
	if (v0_norm < v_max - TRAJ_VEL_EPSILON) {
		d_acc = (v_max * v_max - v0_norm * v0_norm) / (2.0f * acc);
		t_acc = (v_max - v0_norm) / acc;
	} else if (v0_norm > v_max + TRAJ_VEL_EPSILON) {
		d_acc = (v0_norm * v0_norm - v_max * v_max) / (2.0f * acc);
		t_acc = (v0_norm - v_max) / acc;
	}

	float d_dec = (v_max * v_max) / (2.0f * acc); // 从 v_max 减速到0的距离

	/* 3.1 梯形（有匀速段） */
	if (dp_norm >= d_acc + d_dec - TRAJ_POS_EPSILON) {
		/* 速度调整段（加速或减速到 v_max） */
		if (t_acc > TRAJ_TIME_EPSILON) {
			if (v0_norm < v_max) {
				out->segs[out->seg_cnt++] =
					(traj_seg_t){.t = t_acc, .a = move_dir * acc};
			} else {
				out->segs[out->seg_cnt++] =
					(traj_seg_t){.t = t_acc, .a = -move_dir * acc};
			}
		}

		/* 匀速段 */
		float d_const = dp_norm - d_acc - d_dec;
		if (d_const > TRAJ_POS_EPSILON) {
			float t_const = d_const / v_max;
			out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_const, .a = 0.0f};
		}

		/* 减速段 */
		float t_dec = v_max / acc;
		out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_dec, .a = -move_dir * acc};
	}
	/* 3.2 三角形（无匀速段，达不到 v_max） */
	else {
		/* 解算实际能达到的峰值速度 */
		float v_peak_sqr = (dp_norm * 2.0f * acc + v0_norm * v0_norm) / 2.0f;
		if (v_peak_sqr <= 0.0f) {
			return TRAJ_PLAN_ERR_NO_SOLUTION;
		}

		float v_peak = sqrtf(v_peak_sqr);
		if (v_peak > v_max) {
			v_peak = v_max;
		}

		/* 第一段：加速或减速至 v_peak */
		if (v_peak > v0_norm + TRAJ_VEL_EPSILON) {
			float t_acc = (v_peak - v0_norm) / acc;
			out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_acc, .a = move_dir * acc};
		} else if (v0_norm > v_peak + TRAJ_VEL_EPSILON) {
			float t_dec = (v0_norm - v_peak) / acc;
			out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_dec, .a = -move_dir * acc};
		}

		/* 第二段：从 v_peak 减速到 0 */
		float t_dec = v_peak / acc;
		out->segs[out->seg_cnt++] = (traj_seg_t){.t = t_dec, .a = -move_dir * acc};
	}

	return TRAJ_PLAN_OK;
}

/**
 * @brief 初始化规划数据
 *
 * @param d            规划数据结构指针
 * @param start_pos    起始位置
 * @param start_v      起始速度
 * @param acc_max      最大加速度
 * @param exec_cycle   执行周期
 *
 * @return 无
 */
static void traj_plan_init(traj_plan_data_t *d, float start_pos, float start_v, float acc_max,
			   float exec_cycle)
{
	memset(d, 0, sizeof(*d));

	d->plan_in.acc = acc_max;
	d->plan_in.brake_acc = acc_max * 1.2f; // 紧急刹停加速度

	d->plan_in.v = start_v;
	d->pre_pos = start_pos;
	d->act_pos = start_pos;

	/* 初始化紧急停止轨迹段 */
	d->emergency_stop_seg.t = 0.0f;
	d->emergency_stop_seg.a = 0.0f;
}

/**
 * @brief 初始化执行数据
 *
 * @param d            执行数据结构指针
 * @param start_pos    起始位置
 * @param start_vel    起始速度
 * @param start_acc    起始加速度
 *
 * @return 无
 */
static void traj_exec_init(traj_exec_data_t *d, float start_pos, float start_vel, float start_acc)
{
	memset(d, 0, sizeof(*d));
	d->pos = start_pos;
	d->vel = start_vel;
	d->acc = start_acc;
	d->state = TRAJ_EXEC_IDLE;
}

/**
 * @brief 轨迹加载函数
 *
 * @param d            执行数据结构指针
 * @param segs         轨迹段数组
 * @param seg_cnt      轨迹段数量
 *
 * @return 0 表示成功，-1 表示失败
 *
 * @note 边界条件：
 *       - 不修改执行器状态！
 *       - 仅校验参数有效性
 */
static int traj_exec_load(traj_exec_data_t *d, const traj_seg_t *segs, int seg_cnt)
{
	/* 参数校验 */
	if (!segs || seg_cnt <= 0 || seg_cnt > 7) {
		return -1;
	}

	/* 检查轨迹段时间有效性 */
	for (int i = 0; i < seg_cnt; i++) {
		if (segs[i].t < 0.0f || isnan(segs[i].t) || isinf(segs[i].t)) {
			return -1;
		}
	}

	/* 纯数据加载 */
	d->segs = segs;
	d->seg_cnt = seg_cnt;
	d->cur_seg = 0;
	d->seg_time = 0.0f;

	return 0;
}

/**
 * @brief 轨迹执行步进函数
 *
 * @param d            执行数据结构指针
 * @param dt           时间步长
 *
 * @return 无
 *
 * @note 边界条件：
 *       - 纯执行，不做任何状态判断！
 *       - 仅从段中加载数据执行运动学
 *       - 段指针无效时强制结束（硬件保护）
 *       - 执行完毕自动设置END状态
 */
static void traj_exec_step(traj_exec_data_t *d, float dt)
{
	float remain = dt;
	const float time_eps = 1e-7f;

	while (remain > time_eps) {
		/* 最小保护：段指针和索引有效性（硬件保护） */
		if (!d->segs || d->cur_seg < 0 || d->cur_seg >= d->seg_cnt) {
			/* 无有效轨迹，强制结束，加速度归零 */
			d->state = TRAJ_EXEC_END;
			d->acc = 0.0f;
			break;
		}

		const traj_seg_t *s = &d->segs[d->cur_seg];
		float t_left = s->t - d->seg_time;

		/* 当前段已用完 */
		if (t_left <= time_eps) {
			d->seg_time = 0.0f;
			d->cur_seg++;
			continue;
		}

		/* 计算本次使用的时间 */
		float use = (remain < t_left) ? remain : t_left;

		/* 纯运动学更新，无状态判断 */
		d->pos += d->vel * use + 0.5f * s->a * use * use;
		d->vel += s->a * use;
		d->acc = s->a;

		d->seg_time += use;
		remain -= use;

		/* 段结束判断 */
		if (d->seg_time >= s->t - time_eps || fabsf(s->t - d->seg_time) < time_eps) {
			d->seg_time = 0.0f;
			d->cur_seg++;
		}
	}

	/* 所有段执行完毕，设置END状态（唯一的状态修改） */
	if (d->cur_seg >= d->seg_cnt) {
		d->state = TRAJ_EXEC_END;
	}
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
