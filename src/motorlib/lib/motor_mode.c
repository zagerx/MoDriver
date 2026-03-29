#include "motor_mode.h"
#include "statemachine.h"
#include "close_loop.h"
#include "motorlib_control_param.h"

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

void motor_mode_PP(struct statemachine *sm)
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
void motor_mode_PV(struct statemachine *sm)
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
