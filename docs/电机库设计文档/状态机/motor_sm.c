#include "motor_sm.h"
#include "motor.h"

void motor_state_idle(struct statemachine *sm)
{
	struct motor *m = sm->data;
	switch (sm->phase) {
	case sm_phase_enter:
		m->mode = MOTOR_MODE_NONE;
		sm->phase = sm_phase_run;
		break;

	case sm_phase_run:
		if (m->mode != MOTOR_MODE_NONE) {
			sm_trans(sm, motor_state_run);
		}
		break;

	case sm_phase_exit:
		break;
	}
}
void motor_state_run(struct statemachine *sm)
{
	struct motor *m = sm->data;

	switch (sm->phase) {
	case sm_phase_enter:
		sm->phase = sm_phase_run;
		break;

	case sm_phase_run:
		switch (m->mode) {
		case MOTOR_MODE_PROFILE:
			break;
		case MOTOR_MODE_HOME:
			break;
		default:
			sm_trans(sm, motor_state_stop);
			break;
		}
		break;

	case sm_phase_exit:
		break;
	}
}

void motor_state_stop(struct statemachine *sm)
{
	switch (sm->phase) {
	case sm_phase_enter:
		sm->phase = sm_phase_run;
		break;

	case sm_phase_run:
		sm_trans(sm, motor_state_idle);
		break;

	case sm_phase_exit:
		break;
	}
}

void motor_global_estop(struct statemachine *sm)
{
	switch (sm->phase) {
	case sm_phase_enter:
		sm->phase = sm_phase_run;
		break;

	case sm_phase_run:
		break;

	case sm_phase_exit:
		break;
	}
}

void motor_global_falut(struct statemachine *sm)
{
	switch (sm->phase) {
	case sm_phase_enter:
		sm->phase = sm_phase_run;
		break;

	case sm_phase_run:
		break;

	case sm_phase_exit:
		break;
	}
}