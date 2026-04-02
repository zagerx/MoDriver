/*
 * MoDrive 主程序 - CANopen CiA 402 伺服驱动
 */

#include <stddef.h>
#include <stdint.h>
// #include <cstddef>

#include "hardware.h"
#include "motor.h"
#include "stm32g4xx_hal.h"
#include "motorlib_control_param.h"
#include "canopen_app.h"

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

struct motor_param_ext m1_param_ext = {
	.feedback_param =
		{
			.gear_ratio = 1.0f,
			.wheel_radius = 17.5f / 1000.0f, // 17.5mm 轮子半径
		},
	.currsmp_param =
		{
			.gain_phase = PHASE_CURRENT_GAIN, /* 电流采样增益（A/LSB） */
			.gain_i_bus = BUS_CURRENT_GAIN,   /* 母线电流采样增益（A/LSB） */
			.gain_v_bus = BUS_VOLTAGE_GAIN,   /* 母线电压采样增益（V/LSB） */
		},
	.traj_param =
		{
			.acc_max = 10.0f, /* 最大加速度 10 m/s^2 */
			.vmax = 5.0f,     /* 最大速度 5 m/s */
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
			.pos = {.kp = 4000.0f, .ki = 8000.0f, .kd = 0.0f, .limit = 300.0f},

		},
};

/*============================================================================
 * CANopen 应用实例
 *===========================================================================*/

/* 注意：定义为非 static 以便 it.c 访问 */
canopen_app_t canopen_app = {
	.sys_reset_ops = HAL_NVIC_SystemReset,
};

/*============================================================================
 * 主函数
 *===========================================================================*/

int main(void)
{
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

	/* 启动 1ms 定时器中断（一旦开启，永不关闭） */
	HAL_TIM_Base_Start_IT(&htim6);

	/* 初始化 CANopen 应用（节点 ID = 21） */
	if (canopen_app_init(&canopen_app, 21) != 0) {
		/* 初始化失败处理 */
		while (1) {
			HAL_Delay(100);
		}
	}

	/* 绑定 CiA 402 实例参数 */
	cia402_params_bind(&canopen_app.cia402_inst, motor_1, &OD_RAM.x6040_controlword,
			   &OD_RAM.x6041_statusword, &OD_RAM.x6060_modeworld,
			   &OD_RAM.x6061_modeDisplay, &OD_RAM.x60FF_targetVelocity,
			   &OD_RAM.x606C_velocity, &OD_RAM.x603F_errorCode,
			   &OD_RAM.x607A_targetPosition, &OD_RAM.x6071_targetTorque,
			   &OD_RAM.x6064_position, &OD_RAM.x6077_torque);

	cia402_init(&canopen_app.cia402_inst);

	/* 关联电机硬件接口 */
	motor_bind_hardware(motor_1, &m1_hw_ops);

	/* 从外部 flash 读取参数（TODO: 实现 flash 读写） */
	/* flash_read(&m1_param_ext, sizeof(m1_param_ext)); */

	/* 绑定电机参数 */
	motor_bind_param_ext(motor_1, &m1_param_ext);

	/* 初始化电机 */
	motor_init(motor_1);

	/* 开启中断 */
	adc_start();
	tim1_set_adc();

	/* 主循环 */
	uint32_t last_tick = HAL_GetTick();
	while (1) {
		HAL_Delay(1);
		uint32_t current_tick = HAL_GetTick();
		uint32_t dt_ms = current_tick - last_tick;
		last_tick = current_tick;
		canopen_app_process(&canopen_app, dt_ms);
		// cia402_update(&canopen_app.cia402_inst, dt_ms / 1000.0f);
	}
}

/*============================================================================
 * 中断回调函数
 *===========================================================================*/

/**
 * @brief ADC 注入组转换完成回调
 * @note 20kHz 高频控制任务
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	volatile uint16_t raw_uvw[5];
	if (hadc->Instance == ADC1) {
		raw_uvw[0] = (uint32_t)(hadc->Instance->JDR1);
		raw_uvw[1] = (uint32_t)(hadc->Instance->JDR2);
		raw_uvw[2] = (uint32_t)(hadc2.Instance->JDR1);
		raw_uvw[4] = (uint32_t)(hadc->Instance->DR);
		raw_uvw[3] = 0;
		motor_highfreq_task(motor_1, (uint16_t *)raw_uvw);
	}
}

/* 注意：HAL_TIM_PeriodElapsedCallback 定义在 it.c 中，
 * 在那里调用 canopen_app_interrupt(&canopen_app);
 */
