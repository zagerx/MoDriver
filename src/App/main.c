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
static struct feedback_param m1_feedback_param = {
	.wheel_radius = 0.05f,      // 轮子半径 5cm
	.gear_ratio = 10.0f,        // 减速比 10:1
	.pole_pairs = 7.0f,         // 极对数 7
	.direction = 1.0f,          // 正向旋转
	.encoder_resolution = 4096, // 编码器分辨率 4096 CPR
	.encoder_offset = 0,        // 编码器零位偏移
};
static struct motor_param_ext m1_param_ext = {
	.feedback_param = &m1_feedback_param,
};
int main(void)
{
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

	/* 绑定电机硬件接口 */
	motor_bind_hardware(motor_1, &m1_hw_ops);
	motor_bind_param_ext(motor_1, &m1_param_ext);

	motor_init(motor_1);
	/* 主循环 */
	while (1) {
		motor_highfreq_task(motor_1);
		HAL_Delay(1);
		/* TODO: 添加低速任务 */
	}
}
