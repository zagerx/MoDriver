#include "motor.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "statemachine.h"
#include <stdint.h>
#undef NULL
#define NULL (0)

/* 控制周期 100us (10kHz) */
#define CONTROL_PERIOD_DT 0.0001f

static volatile uint16_t test_value2;
static volatile uint16_t test_value = 0;

void motor_idle_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	switch (sm->phase) {
	case ENTER:
		test_value = 1;
		sm->phase = RUNING;
		break;
	case RUNING:
		test_value = 2;
		break;
	case EXIT:
		test_value = 3;
		break;
	default:
		break;
	}
}
void motor_runing_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	switch (sm->phase) {
	case ENTER:
		test_value = 0;
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

void motor_bind_hardware(struct motor *motor, const struct motor_hw_ops *hw)
{
	if (!motor || !hw) {
		return;
	}

	if (hw->encoder) {
		feedback_bind_encoder(motor->feedback, hw->encoder);
	}

	if (hw->inverter) {
		inverter_bind_inverter(motor->inverter, hw->inverter);
	}
}

void motor_init(struct motor *motor)
{
	struct statemachine *sm = motor->sm;
	statemachine_init(sm, motor, motor_idle_state, NULL, 0);
}
void motor_highfreq_task(struct motor *motor)
{
	if (!motor || !motor->config) {
		return;
	}
	if (motor->feedback) {
		feedback_update(motor->feedback, CONTROL_PERIOD_DT);
	}
	struct statemachine *sm = motor->sm;

	if (test_value2++ > 3000 && test_value != 0) {
		TRAN_STATE(sm, motor_runing_state);
	}
	sm_dispatch(sm);
}
