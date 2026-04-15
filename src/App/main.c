/*
 * MoDrive 主程序
 */

#include <stddef.h>
#include <stdint.h>

#include "hardware.h"
#include "motor.h"
#include "motor_interface_params.h"
#include "canopen_app/canopen_app.h"
#include "OD.h"
/*============================================================================
 * 电机 1 硬件接口配置
 *===========================================================================*/

static const struct encoder_ops m1_encoder_ops = {
	.read = encoder_getraw,
};
static const struct inverter_ops m1_inverter_ops = {
	.enable = tim1_pwm_enable,
	.disable = tim1_pwm_disable,
	.set_voltage = tim1_pwm_set_duty,
};
static const struct motor_hw_ops m1_hw_ops = {
	.encoder = &m1_encoder_ops,
	.inverter = &m1_inverter_ops,
};
struct motor_param_ext *pmotor1_param =
	(struct motor_param_ext *)(&OD_RAM.x2009_motorlib_params.wheel_radius);
/* 注意：定义为非 static 以便 it.c 访问 */
struct canopen_app canopen_app = {
	.sys_reset_ops = HAL_NVIC_SystemReset,
};

int main(void)
{
	// SCB->VTOR = 0x08003000;
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

	/* 初始化 CANopen 应用（节点 ID = 21） */
	if (canopen_app_init(&canopen_app, motor_1) != 0) {
		/* 初始化失败处理 */
		while (1) {
			HAL_Delay(100);
		}
	}

	/* 关联电机硬件接口 */
	motor_bind_hardware(motor_1, &m1_hw_ops);

	/* 绑定电机参数 */
	motor_bind_param_ext(motor_1, pmotor1_param);

	/* 初始化电机 */
	motor_init(motor_1);

	hardware_start_irq();
	/* 主循环 */
	uint32_t last_tick = HAL_GetTick();
	while (1) {
		HAL_Delay(1);
		uint32_t current_tick = HAL_GetTick();
		uint32_t dt_ms = current_tick - last_tick;
		last_tick = current_tick;
		canopen_app_process(&canopen_app, dt_ms);
	}
}
