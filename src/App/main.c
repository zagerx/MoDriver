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
	.enable = tim1_pwm_enable,
	.disable = tim1_pwm_disable,
	.set_voltage = tim1_pwm_set_duty,
};

static const struct motor_hw_ops m1_hw_ops = {
	.encoder = &m1_encoder_ops,
	.inverter = &m1_inverter_ops,
};
static struct feedback_param m1_feedback_param = {
	0
	// .wheel_radius = 0.05f,      // 轮子半径 5cm
	// .gear_ratio = 10.0f,        // 减速比 10:1
	// .pole_pairs = 7.0f,         // 极对数 7
	// .direction = 1.0f,          // 正向旋转
	// .encoder_resolution = 4096, // 编码器分辨率 4096 CPR
	// .encoder_offset = 0,        // 编码器零位偏移
};
static struct currsmp_param m1_currsmp_param = {
	// .a_chn_offset = 2048, // A相ADC通道偏移
	// .b_chn_offset = 2048, // B相ADC通道偏移
	// .c_chn_offset = 2048, // C相ADC通道偏移
	.gain_phase = 0.006011f,  // 电流采样增益（A/LSB）
	.gain_i_bus = 0.01f,      // 母线电流采样增益（A/LSB）
	.gain_v_bus = 0.0112793f, // 母线电压采样增益（V/LSB） 130V(130+10)
};
static struct motor_param_ext m1_param_ext = {
	.feedback_param = &m1_feedback_param,
	.currsmp_param = &m1_currsmp_param,
};
int main(void)
{
	/* 初始化硬件层（时钟、GPIO、外设等） */
	hardware_init();

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
