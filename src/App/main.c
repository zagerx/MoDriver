/*
 * MoDrive 主程序 - CANopen CiA 402 伺服驱动
 */

#include <stddef.h>
#include <stdint.h>

#include "hardware.h"
#include "motor.h"
#include "motorlib_control_param.h"
#include "canopen_app/canopen_app.h"
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
struct motor_param_ext m1_param_ext = {
	.feedback_param =
		{
			.gear_ratio = FEEDBACK_GEAR_RATIO,
			.wheel_radius = FEEDBACK_WHEEL_RADIO, // 17.5mm 轮子半径
		},
	.currsmp_param =
		{
			.gain_phase = PHASE_CURRENT_GAIN, /* 电流采样增益（A/LSB） */
			.gain_i_bus = BUS_CURRENT_GAIN,   /* 母线电流采样增益（A/LSB） */
			.gain_v_bus = BUS_VOLTAGE_GAIN,   /* 母线电压采样增益（V/LSB） */
		},
	.traj_param =
		{
			.acc_max = TRAJAC_MAX_ACC,
			.vmax = TRAJAC_MAX_VEL,
		},
	.foc_param =
		{
			.d_axis = {.kp = CURRMENT_LOOP_KP,
				   .ki = CURRMENT_LOOP_KI,
				   .kd = 0.0f,
				   .limit = CURRMENT_LOOP_LIMIT},
			.q_axis = {.kp = CURRMENT_LOOP_KP,
				   .ki = CURRMENT_LOOP_KI,
				   .kd = 0.0f,
				   .limit = CURRMENT_LOOP_LIMIT},
			.vel = {.kp = SPEED_LOOP_KP,
				.ki = SPEED_LOOP_KI,
				.kd = 0.0f,
				.limit = SPEED_LOOP_LIMIT},
			.pos = {.kp = POSITION_LOOP_KP,
				.ki = POSITION_LOOP_KI,
				.kd = 0.0f,
				.limit = POSITION_LOOP_LIMIT},
		},
};

/* 注意：定义为非 static 以便 it.c 访问 */
canopen_app_t canopen_app = {
	.sys_reset_ops = HAL_NVIC_SystemReset,
};

int main(void)
{
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

	/* 从外部 flash 读取参数（TODO: 实现 flash 读写） */
	/* flash_read(&m1_param_ext, sizeof(m1_param_ext)); */

	/* 绑定电机参数 */
	motor_bind_param_ext(motor_1, &m1_param_ext);

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
