/**
 * @file close_loop.c
 * @brief 电机闭环控制实现
 * @details 实现电机三环控制（位置环、速度环、电流环）的PI/PID控制算法
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
 * @param[in] dt 时间步长（单位：秒）
 * @details 执行位置闭环控制，读取轨迹规划位置并与实际位置做PID运算，输出速度指令
 * @note 位置环输出 = 位置PID计算结果 + 轨迹规划速度前馈（前馈系数为1）
 */
void motor_position_loop(struct motor *motor, float dt)
{
	/* 获取FOC相关结构体指针 */
	struct foc *foc = &motor->foc;
	struct foc_pid *position_pi = &foc->ctrl.position;
	struct foc_measurement *meas = &foc->meas;
	struct trajectory_plan *traj_plan = &motor->traj_plan;
	
	/* 获取轨迹规划的位置和速度（单位转换：mm -> m） */
	float plan_position = trajectory_planner_get_pos(traj_plan);
	float plan_velocity = trajectory_planner_get_vel(traj_plan) / (17.5f / 1000.0f);
	float current_pos = meas->fd_out->odometer;
	
	/* 调试用变量：记录目标位置、实际位置、规划速度 */
	static volatile float test_tar_pos, test_real_pos, test_plann_vel;
	(void)test_tar_pos;
	(void)test_real_pos;
	(void)test_plann_vel;
	test_tar_pos = plan_position * 1000.0f;
	test_real_pos = current_pos * 1000.0f;
	test_plann_vel = plan_velocity * 1000.0f;
	
	/* 位置环PID计算，输出作为速度环的目标输入 */
	float temp = foc_pid_run(position_pi, plan_position, current_pos, dt);
	foc->ref.velocity = temp + plan_velocity; /* 速度前馈：轨迹规划的速度 */
}

/**
 * @brief 复位位置环控制器
 * @param[in] motor 电机实例指针
 * @details 复位位置环、速度环、电流环的PID控制器状态，并清零参考值
 * @note 通常在模式切换或故障恢复时调用，确保各环从稳定状态开始
 */
void motor_position_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *position_pi = &foc->ctrl.position;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_pid *current_d_pi = &foc->ctrl.d_axis;
	struct foc_pid *current_q_pi = &foc->ctrl.q_axis;
	
	/* 清零参考值 */
	foc->ref.velocity = 0.0f;
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	
	/* 复位各环PID控制器 */
	foc_pid_reset(position_pi);
	foc_pid_reset(velocity_pi);
	foc_pid_reset(current_d_pi);
	foc_pid_reset(current_q_pi);
}

/**
 * @brief 电机速度闭环控制
 * @param[in] motor 电机实例指针
 * @param[in] target_vel 目标速度（单位：rad/s）
 * @details 执行速度环PI控制，根据目标速度和实际速度计算q轴电流参考值，
 *          并将d轴电流参考值设为0（Id=0控制，最大化转矩电流比）
 * @note 速度环输出为q轴电流参考值，受电流限幅限制
 */
void motor_velocity_loop(struct motor *motor, float target_vel)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_measurement *meas = &foc->meas;

	/* 获取实际速度（单位：rad/s） */
	float vel = meas->fd_out->velocity_rad_s;

	/* 速度环PI计算，输出q轴电流参考值 */
	foc->ref.i_q = foc_pid_run(velocity_pi, target_vel, vel, SPEED_PERIOD_DT);
	foc->ref.i_d = 0.0f; /* Id=0控制策略 */
}

/**
 * @brief 复位速度环控制器
 * @param[in] motor 电机实例指针
 * @details 复位速度环和电流环的PID控制器状态，并清零电流参考值
 * @note 通常在速度环退出或故障恢复时调用，确保速度环和电流环从稳定状态开始
 */
void motor_velocity_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *velocity_pi = &foc->ctrl.velocity;
	struct foc_pid *current_d_pi = &foc->ctrl.d_axis;
	struct foc_pid *current_q_pi = &foc->ctrl.q_axis;
	
	/* 清零电流参考值 */
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	
	/* 复位速度环和电流环PID控制器 */
	foc_pid_reset(velocity_pi);
	foc_pid_reset(current_d_pi);
	foc_pid_reset(current_q_pi);
}

/**
 * @brief 电机电流环控制
 * @param[in] motor 电机实例指针
 * @details 执行d/q轴电流环PID控制：
 *          1. d/q轴电流PID计算输出电压
 *          2. 电压限幅（根据母线电压计算最大输出电压）
 *          3. 坐标变换后通过SVPWM计算三相占空比
 *          4. 输出到逆变器
 * @note 电流环是三环控制的最内环，执行频率最高（通常10kHz）
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
	
	/* d/q轴电流环PID计算，输出电压指令 */
	ud = foc_currentloop_pid_run(d_axis_pid, foc->ref.i_d, meas->i_d, CONTROL_PERIOD_DT);
	uq = foc_currentloop_pid_run(q_axis_pid, foc->ref.i_q, meas->i_q, CONTROL_PERIOD_DT);
	
	/* 记录原始电压值用于限幅计算 */
	float ud_limit = ud;
	float uq_limit = uq;

	/* 获取母线电压和电角度 */
	float vbus = meas->cs_out->v_bus;
	float eangle = meas->fd_out->eangle_rad;
	
	/* 电压限幅（考虑SVPWM最大调制比） */
	svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
	
	/* 更新PID积分限幅（抗积分饱和） */
	foc_currentpid_saturation(&foc->ctrl.d_axis, ud_limit, ud);
	foc_currentpid_saturation(&foc->ctrl.q_axis, uq_limit, uq);
	
	/* SVPWM归一化计算 */
	svpwm_normalize(eangle, vbus, ud_limit, uq_limit, &ualpha, &ubeta);
	
	/* 计算三相占空比 */
	svpwm_calc_duty(ualpha, ubeta, duty);
	
	/* 输出到逆变器 */
	inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
}

/**
 * @brief 复位电流环控制器
 * @param[in] motor 电机实例指针
 * @details 复位d/q轴电流环的PID控制器状态，并清零电流参考值
 * @note 通常在电流环退出或逆变器关闭时调用，确保电流环从稳定状态开始
 */
void motor_current_loop_reset(struct motor *motor)
{
	struct foc *foc = &motor->foc;
	struct foc_pid *d_axis_pid = &foc->ctrl.d_axis;
	struct foc_pid *q_axis_pid = &foc->ctrl.q_axis;
	
	/* 清零电流参考值 */
	foc->ref.i_q = 0.0f;
	foc->ref.i_d = 0.0f;
	
	/* 复位d/q轴电流环PID控制器 */
	foc_pid_reset(d_axis_pid);
	foc_pid_reset(q_axis_pid);
}
