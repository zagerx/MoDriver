#include "svpwm.h"
#include "coord_transform.h"
extern void openloop_voltage_apply_dq(float ud, float uq, float elec_angle, float vbus);

void test_main(void)
{
	static volatile uint32_t test_count = 0;
	static volatile float test_self_eangle = 0.0f;

	// if (test_count++ < 5000) {
	// 	openloop_voltage_apply_dq(0.2f, 0.0f, test_self_eangle, 24.0f);
	// 	return;
	// }
	// test_count = 5001;
	test_self_eangle += 0.0008f;
	if (test_self_eangle >= 2.0f * 3.14159265358979323846f) {
		test_self_eangle = 0.0f;
	}
	openloop_voltage_apply_dq(2.8f, 0.0f, test_self_eangle, 24.0f);
}
