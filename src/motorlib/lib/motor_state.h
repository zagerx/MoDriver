#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H
#include "statemachine.h"
void motor_init_state(struct statemachine *sm);
void motor_carib_state(struct statemachine *sm);
void motor_idle_state(struct statemachine *sm);
void motor_runing_state(struct statemachine *sm);

#endif /* MOTOR_STATE_H */