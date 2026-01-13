#include "motor.h"
#include <stdint.h>

static struct motor motor1;

// adc中断调用
void adc_isq_10KHZ(void)
{
	struct motor *motor = &motor1;
	uint16_t fault_code; // 由别处更改，暂不实现
	if (fault_code) {
		// 设置电机故障
		motor_fault(motor);
	} else {
		motor_clear_fault(motor);
	}
	motor_sm_dispatch(&motor1);
}

void app_init(void)
{
	motor_sm_init(&motor1);
}

// app层，周期性调用
void app_task(void)
{
	uint8_t cmd;
	// cmd从队列或者其他地方更改，暂不实现
	switch (cmd) {
	case 0: { // 停止命令
		motor_stop(&motor1);
	} break;
	case 1: {

	} break;
	default:
		break;
	}
}