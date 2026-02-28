#include "motor.h"
#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
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

void motor_highfreq_task(struct motor *motor)
{
	if (!motor || !motor->config) {
		return;
	}

	if (motor->feedback) {
		feedback_update(motor->feedback);
	}
}
