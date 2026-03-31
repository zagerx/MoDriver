#include "motor_mode.h"
#include "statemachine.h"
#include "close_loop.h"
#include "motorlib_control_param.h"
#include "_motorlib_internal.h"
void motor_mode_none(struct statemachine *sm)
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

void motor_mode_PP(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct trajectory_plan *traj_plan = &motor->traj_plan;
	struct foc *foc = &motor->foc;
	float start_pos = foc->meas.fd_out->odometer; /* 当前位置作为起始位置 */
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		sm->count = 0;
		trajectory_planner_init(traj_plan, start_pos, 0.0f, 0.0f, POSITION_PERIOD_DT);
		break;

	case RUNING:
		if (sm->count >= POSITION_LOOP_INTERVAL) {
			sm->count = 0;
			motor_position_loop(motor, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)(SPEED_LOOP_INTERVAL) == 0) {
			motor_velocity_loop(motor);
		}
		motor_currment_loop(motor);
		sm->count++;
		break;

	case EXIT:
		break;

	default:
		break;
	}
}
void motor_mode_PV(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};

	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;

	switch (sm->phase) {
	case ENTER:
		sm->count = 0;
		sm->phase = RUNING;
		break;

	case RUNING:
		if (sm->count++ > SPEED_LOOP_INTERVAL) {
			sm->count = 0;

			motor_velocity_loop(motor);
		}
		motor_currment_loop(motor);
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

void motor_mode_HOMING(struct statemachine *sm)
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

void _tran_mode(struct motor *motor, enum motor_mode new_mode)
{
	struct statemachine *sm_mode = &motor->sm_mode;
	switch (new_mode) {
	case MODE_NONE:
		if (sm_mode->current_state != motor_mode_none) {
			TRAN_STATE(sm_mode, motor_mode_none);
		}
		break;

	case MODE_PP:
		if (sm_mode->current_state != motor_mode_PP) {
			TRAN_STATE(sm_mode, motor_mode_PP);
		}

		break;

	case MODE_PV:
		if (sm_mode->current_state != motor_mode_PV) {
			TRAN_STATE(sm_mode, motor_mode_PV);
		}
		break;

	case MODE_HM:
		if (sm_mode->current_state != motor_mode_HOMING) {
			TRAN_STATE(sm_mode, motor_mode_HOMING);
		}
		break;

	default:
		break;
	}
}