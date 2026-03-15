#include "foc_pid.h"
#include "statemachine.h"
#include "_motorlib_internal.h"
#include "calibration.h"
#include "inverter.h"
#include "motor_state.h"
#include <stdint.h>
#include "motorlib_control_param.h"
#include "svpwm.h"
void motor_openloop_encoder_state(struct statemachine *sm);

/**
 * motor_carib_state - 校准状态
 * @sm: 状态机实例
 *
 * 电机参数异常时进入的校准状态。
 * 调用 calibration_task 执行校准，校准模块内部直接操作逆变器。
 */
void motor_carib_state(struct statemachine *sm)
{
	enum {
		CALIBRATING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	enum calibration_status calib_status;

	switch (sm->phase) {
	case ENTER:
		/* 进入校准状态 */
		calibration_init(motor);
		sm->phase = CALIBRATING;
		break;

	case CALIBRATING:
		/* 调用校准任务，由校准模块自主控制逆变器等硬件 */
		calib_status = calibration_task(motor);

		/* 根据校准结果迁移状态 */
		if (calib_status == CALIBRATION_STATUS_SUCCESS) {
			TRAN_STATE(sm, motor_runing_state);
		} else if (calib_status == CALIBRATION_STATUS_FAILED) {
			/* 校准失败 */
			TRAN_STATE(sm, motor_init_state);
		}
		break;

	case EXIT:
		/* 退出校准状态，确保逆变器禁用（安全考虑） */
		/* 注：即使校准模块内部已禁用，这里再禁一次确保万无一失 */
		if (motor->inverter) {
			inverter_disable(motor->inverter);
		}
		break;

	default:
		break;
	}
}
/**
 * motor_idle_state - 空闲状态
 * @sm: 状态机实例
 *
 * 电机初始化后的默认状态，等待运行指令
 */
void motor_init_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}
}

/**
 * motor_idle_state - 空闲状态
 * @sm: 状态机实例
 *
 * 电机初始化后的默认状态，等待运行指令
 */
void motor_idle_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}
}

/**
 * motor_runing_state - 运行状态
 * @sm: 状态机实例
 *
 * 电机正常运行状态，执行FOC控制循环
 */
void motor_runing_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = motor->inverter;
	struct foc_data *foc_data = &motor->foc.data;
	struct foc_measurement *meas = &foc_data->meas;
	struct foc_control *ctrl = &foc_data->ctrl;
	struct foc_pid *d_axis_pid = &ctrl->d_axis;
	struct foc_pid *q_axis_pid = &ctrl->q_axis;
	struct foc_pid *vel_pid = &ctrl->velocity;
	struct feedback *feedback = motor->feedback;
	float ud, uq; // d轴电压为0，保持固定角度
	float ualpha, ubeta;
	float duty[3];
	float vbus = meas->currsmp->v_bus;

	float eangle = feedback_get_elec_angle(feedback);

	switch (sm->phase) {
	case ENTER:
		inverter_enable(inverter);
		sm->phase = RUNING;
		sm->count = 0;
		foc_pid_init(vel_pid, ctrl->velocity.params->kp, ctrl->velocity.params->ki,
			     ctrl->velocity.params->limit);
		foc_pid_init(d_axis_pid, ctrl->d_axis.params->kp, ctrl->d_axis.params->ki,
			     ctrl->d_axis.params->limit);
		foc_pid_init(q_axis_pid, ctrl->q_axis.params->kp, ctrl->q_axis.params->ki,
			     ctrl->q_axis.params->limit);
		break;
	case RUNING:
		if (sm->count++ > SPEED_LOOP_INTERVAL) {
			sm->count = 0;
			float target = motor->param_ext->foc_param.target_pos;

			foc_data->ref.i_q = foc_pid_run(
				vel_pid, target, meas->feeback->velocity_rad_s, SPEED_PERIOD_DT);
			// foc_data->ref.i_d = 0.0f;
			foc_data->ref.i_d = 0.0f;
			// foc_data->ref.i_q = target;
		}
		ud = foc_currentloop_pid_run(&foc_data->ctrl.d_axis, foc_data->ref.i_d, meas->i_d,
					     CONTROL_PERIOD_DT);
		uq = foc_currentloop_pid_run(&foc_data->ctrl.q_axis, foc_data->ref.i_q, meas->i_q,
					     CONTROL_PERIOD_DT);
		float ud_limit = ud;
		float uq_limit = uq;
		svpwm_limit_voltage(vbus, &ud_limit, &uq_limit);
		foc_currentpid_saturation(&foc_data->ctrl.d_axis, ud_limit, ud);
		foc_currentpid_saturation(&foc_data->ctrl.q_axis, uq_limit, uq);
		svpwm_normalize(eangle, vbus, ud_limit, uq_limit, &ualpha, &ubeta);
		svpwm_calc_duty(ualpha, ubeta, duty);
		inverter_set_voltage(motor->inverter, duty[0], duty[1], duty[2]);
		break;
	case EXIT:
		break;
	default:
		break;
	}
}
void motor_openloop_encoder_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	struct inverter *inverter = motor->inverter;
	static float target;
	// struct feedback *feedback = motor->feedback;
	static uint32_t debug_cnt = 0;

	switch (sm->phase) {
	case ENTER:
		inverter_enable(inverter);
		target = 1.0f;
		sm->phase = RUNING;
		break;
	case RUNING:
		target = motor->param_ext->foc_param.target_pos;
		open_loop_encoder(motor, target);
		// if (++debug_cnt % 500 == 0) {
		// 	debug_cnt = 0;
		// 	target = -target;                            // 反转目标位置
		// 	foc_pid_reset(&motor->foc.data.ctrl.d_axis); // 重置PID控制器状态
		// }
		// currment_debug(motor, target);

		break;
	case EXIT:
		inverter_disable(inverter);
		break;
	default:
		break;
	}
}
