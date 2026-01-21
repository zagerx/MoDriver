#include "svpwm.h"
#include "coord_transform.h"
#include "tim.h"
#undef M_PI
#define M_PI 3.14159265358979323846f
#undef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)

#ifndef M_TWOPI
#define M_TWOPI (2.0f * 3.14159265358979323846f)
#endif

void openloop_voltage_apply_dq(float ud, float uq, float elec_angle, float vbus)
{
	float sin_val, cos_val;
	float ualpha, ubeta;
	float duty[3];
	ud *= (1.0f / (vbus * 0.57735f));
	uq *= (1.0f / (vbus * 0.57735f));
	/* dq -> αβ */
	sin_cos_f32(elec_angle * RAD_TO_DEG, &sin_val, &cos_val);
	inv_park_f32(ud, uq, &ualpha, &ubeta, sin_val, cos_val);

	/* αβ -> PWM */
	svpwm_seven_segment(ualpha, ubeta, &duty[0], &duty[1], &duty[2]);
	/* Apply to inverter */
	// inverter_set_3phase_voltages(inverter, duty[0], duty[1], duty[2]);
	tim_set_pwm(duty[0], duty[1], duty[2]);
}
