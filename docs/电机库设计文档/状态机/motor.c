#include "motor.h"
#include "motor_sm.h"
void motor_sm_init(struct motor *m)
{
	sm_init(&m->sm, motor_state_idle, m);
}
void motor_sm_dispatch(struct motor *motor)
{
	sm_dispatch(&motor->sm);
}

void motor_stop(struct motor *motor)
{
	sm_trans(&(motor->sm), motor_state_stop);
}
void motor_fault(struct motor *motor)
{
	sm_set_global(&motor->sm, 0, motor_global_falut);
}

void motor_clear_fault(struct motor *motor)
{
	sm_clear_global(&motor->sm, 0);
}