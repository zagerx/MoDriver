#include "motor.h"
#include "_motorlib_internal.h"

void motor_higfre_task(struct motor *motor)
{
	(void)motor;
	feedback_update(motor->config->feedback);
}

void motor_register_callback(struct motor *motor, uint16_t (*cb)(void), void (*disable)(void),
			     void (*enable)(void), void (*set)(float, float, float))
{
	struct feedback *feedback = motor->config->feedback;
	struct inverter *inverter = motor->config->inverter;
	feedback_register_callback(feedback, cb);
	inverter_register_callback(inverter, disable, enable, set);
}
