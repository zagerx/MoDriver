#include <stddef.h>
#include <stdint.h>

#include "hardware.h"
#include "motor.h"
#include "motor_driver.h"
#include "stm32g4xx_hal.h"
/* 电机1硬件接口定义 */
static const struct encoder_ops m1_encoder_ops = {
	.read = encoder_getraw,
};

static const struct inverter_ops m1_inverter_ops = {
	.enable = NULL,
	.disable = NULL,
	.set_voltage = NULL,
};

static const struct motor_hw_ops m1_hw_ops = {
	.encoder = &m1_encoder_ops,
	.inverter = &m1_inverter_ops,
};
int main(void)
{
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

	/* 绑定电机硬件接口 */
	motor_bind_hardware(motor_1, &m1_hw_ops);
	motor_init(motor_1);
	/* 主循环 */
	while (1) {
		motor_highfreq_task(motor_1);
		HAL_Delay(1);
		/* TODO: 添加低速任务 */
	}
}
