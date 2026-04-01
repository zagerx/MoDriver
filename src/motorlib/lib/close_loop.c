/**
 * @file close_loop.c
 * @brief 电机闭环控制实现
 */

#include "close_loop.h"
#include "currsmp.h"
#include "feedback.h"
#include "foc.h"
#include "inverter.h"
#include "_motorlib_internal.h"
#include "foc_pid.h"
#include "svpwm.h"
#include "motorlib_control_param.h"
/**
 * @brief 电机位置环控制
 * @param[in] motor 电机实例指针
 * @param[in] dt 时间步长
 * @details 执行位置闭环控制，读取轨迹规划位置并与实际位置做PID运算，输出速度指令
 */
void motor_position_loop(struct motor *motor, float dt)
{
	/* 位置环控制逻辑 */
	struct foc *foc = &motor->foc;
	struct foc_pid *position_pi = &foc->ctrl.position;
	struct foc_measurement *meas = &foc->meas;
	struct trajectory_plan *traj_plan = &motor->traj_plan;
	float plan_position = trajectory_planner_get_pos(traj_plan);
	float plan_velocity = trajectory_planner_get_vel(traj_plan) / (17.5f / 1000.0f);
	float current_pos = meas->fd_out->odometer;
	static volatile float test_tar_pos, test_real_pos, test_plann_vel;
	(void)test_tar_pos;
	(void)test_real_pos;
	(void)test_plann_vel;
	test_tar_pos = plan_position * 1000.0f;
	test_real_pos = current_pos * 1000.0f;
	test_plann_vel = plan_velocity * 1000.0f;
	float temp = foc_pid_run(position_pi, plan_position, current_pos, dt);
	/* 将位置环输出作为速度环的目标输入 */
	foc->ref.velocity = temp + plan_velocity; /* 前馈项：轨迹规划的速度 */
}
void motor_position_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *position_pi = &foc->ctrl.position;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_pid *current_d_pi = &foc->ctrl.d_axis;
	struct foc_pid *current_q_pi = &foc->ctrl.q_axis;
	foc->ref.velocity = 0.0f;
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	foc_pid_reset(position_pi);
	foc_pid_reset(velocity_pi);
	foc_pid_reset(current_d_pi);
	foc_pid_reset(current_q_pi);
}
/**
 * @brief 电机速度闭环控制
 *
 * 执行速度环PI控制，根据目标速度和实际速度计算q轴电流参考值，
 * 并将d轴电流参考值设为0。
 *
 * @param[in,out] motor 电机对象指针，包含FOC参数、反馈信息和控制器状态
 *
 * @return 无返回值
 */
void motor_velocity_loop(struct motor *motor, float target_vel)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_measurement *meas = &foc->meas;

	float vel = meas->fd_out->velocity_rad_s;

	foc->ref.i_q = foc_pid_run(velocity_pi, target_vel, vel, SPEED_PERIOD_DT);
	foc->ref.i_d = 0.0f;
}
void motor_velocity_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_pid *current_d_pi = &foc->ctrl.d_axis;
	struct foc_pid *current_q_pi = &foc->ctrl.q_axis;
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	foc_pid_reset(velocity_pi);
	foc_pid_reset(current_d_pi);
	foc_pid_reset(current_q_pi);
}

/**
 * @brief 电机电流环控制
 * @param[in] motor 电机实例指针
 * @details 执行d/q轴电流环PID控制，经SVPWM变换后输出三相占空比到逆变器
 */
void motor_currment_loop(struct motor *motor)
{

	struct foc *foc = &motor->foc;

	struct foc_measurement *meas = &foc->meas;
	struct foc_control *ctrl = &foc->ctrl;
	struct foc_pid *d_axis_pid = &ctrl->d_axis;
	struct foc_pid *q_axis_pid = &ctrl->q_axis;
	float ud, uq;
	float ualpha, ubeta;
	float duty[3];
	ud = foc_currentloop_pid_run(d_axis_pid, foc->ref.i_d, meas->i_d, CONTROL_PERIOD_DT);
	uq = foc_currentloop_pid_run(q_axis_pid, foc->ref.i_q, meas->i_q, CONTROL_PERIOD_DT);
	float ud_limit = ud;
	float uq_limit = uq;

	float vbus = meas->cs_out->v_bus;
	float eangle = meas->fd_out->eangle_rad;
	svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
	foc_currentpid_saturation(&foc->ctrl.d_axis, ud_limit, ud);
	foc_currentpid_saturation(&foc->ctrl.q_axis, uq_limit, uq);
	svpwm_normalize(eangle, vbus, ud_limit, uq_limit, &ualpha, &ubeta);
	svpwm_calc_duty(ualpha, ubeta, duty);
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}
void motor_current_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *d_axis_pid = &foc->ctrl.d_axis;
	struct foc_pid *q_axis_pid = &foc->ctrl.q_axis;
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	foc_pid_reset(d_axis_pid);
	foc_pid_reset(q_axis_pid);
}
