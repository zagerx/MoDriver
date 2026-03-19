#include <stddef.h>
#include <stdint.h>

#include "hardware.h"
#include "motor.h"
#include "motor_driver.h"
#include "stm32g4xx_hal.h"
#include "motorlib_control_param.h"
/* 电机1硬件接口定义 */
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
	.feedback_param = {0},
	.currsmp_param =
		{
			.gain_phase = PHASE_CURRENT_GAIN, // 电流采样增益（A/LSB）
			.gain_i_bus = BUS_CURRENT_GAIN,   // 母线电流采样增益（A/LSB）
			.gain_v_bus = BUS_VOLTAGE_GAIN,   // 母线电压采样增益（V/LSB） 130V(130+10)
		},
	.traj_param =
		{
			.acc_max = 10.0f, // 最大加速度 10 m/s^2
			.vmax = 5.0f,     // 最大速度 5 m/s},
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
			.pos = {.kp = 0.1f, .ki = 1.0f, .kd = 0.0f, .limit = 100.0f},
			.target_pos = 0.0f,
			.target_vel = 0.0f,
			.target_torque = 0.0f,
		},
};

extern int canopen_app_init(void);
extern void canopen_app_process(void);

int main(void)
{
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

	canopen_app_init();

	/* 绑定电机硬件接口 */
	motor_bind_hardware(motor_1, &m1_hw_ops);

	/*从外部flash读取参数*/
	// flash_read(&m1_param_ext, sizeof(m1_param_ext));//TODO: 实现flash读写接口并使用

	/* 绑定电机参数 */
	motor_bind_param_ext(motor_1, &m1_param_ext);

	motor_init(motor_1);

	/*开启中断*/
	adc_start();
	tim1_set_adc();
	/* 主循环 */
	while (1) {
		// motor_highfreq_task(motor_1);
		HAL_Delay(1);
		canopen_app_process();
		/* TODO: 添加低速任务 */
	}
}
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
