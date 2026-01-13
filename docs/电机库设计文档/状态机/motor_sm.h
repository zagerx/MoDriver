#ifndef _MOTOR_SM_H
#define _MOTOR_SM_H

#include "statemachine.h"

void motor_state_idle(struct statemachine *sm);
void motor_state_run(struct statemachine *sm);
void motor_state_stop(struct statemachine *sm);
void motor_state_estop(struct statemachine *sm);
void motor_global_falut(struct statemachine *sm);

#endif