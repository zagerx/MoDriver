#ifndef MOTOR_H
#define MOTOR_H
#include "statemachine.h"

enum motor_mode {
	MOTOR_MODE_NONE = 0,
	MOTOR_MODE_PROFILE,
	MOTOR_MODE_HOME,
};

struct motor {
	struct statemachine sm;
	enum motor_mode mode;
};

void motor_sm_init(struct motor *m);
void motor_sm_dispatch(struct motor *motor);
void motor_stop(struct motor *motor);
void motor_fault(struct motor *motor);
void motor_clear_fault(struct motor *motor);
#endif
