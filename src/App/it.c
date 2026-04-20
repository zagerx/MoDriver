#include "motor.h"
#include "motor_interface_mode.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
extern uint8_t sg_uartreceive_buff[125];
// extern struct motor_param_ext m1_param_ext;
extern struct motor_param_ext *pmotor1_param;

void process_data(uint8_t *data, uint16_t len);

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if (USART1 == huart->Instance) {
		if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			HAL_UART_DMAStop(huart);
			volatile unsigned short data_length =
				sizeof(sg_uartreceive_buff) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			process_data(sg_uartreceive_buff, data_length);
			memset(sg_uartreceive_buff, 0, 125);
			HAL_UART_Receive_DMA(huart, (uint8_t *)sg_uartreceive_buff,
					     sizeof(sg_uartreceive_buff));
		}
	}
}

enum foc_data_index {
	INDEX_TAR = 0,
	INDEX_VP_PI,
	INDEX_STATE,
	INDEX_MODE,
	INDEX_MAX,
};

// 命令映射表结构
typedef struct {
	const char *cmd_name;
	uint8_t min_params; // 最少需要的参数个数
	enum foc_data_index data_index;
} command_map_t;

// 命令表定义
static const command_map_t cmd_map[] = {
	{"tar", 2, INDEX_TAR},
	{"pid", 2, INDEX_VP_PI},
	{"state", 1, INDEX_STATE},
	{"mode", 1, INDEX_MODE},
};
void process_data(uint8_t *data, uint16_t len)
{

	if (data[0] == 0 || len == 0 || len > 255) {
		return;
	}

	char buf[256];
	uint16_t copy_len = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
	memcpy(buf, (char *)data, copy_len);
	buf[copy_len] = '\0';

	// 找冒号
	char *colon_pos = strchr(buf, ':');
	if (!colon_pos) {
		return;
	}

	*colon_pos = '\0';
	char *cmd = buf;
	char *params_str = colon_pos + 1;

	// ========== 高效参数解析 ==========
	float params[10];
	uint8_t param_count = 0;
	char *ptr = params_str;

	while (*ptr != '\0' && param_count < 10) {
		// 跳过空格
		while (*ptr == ' ') {
			ptr++;
		}
		if (*ptr == '\0') {
			break;
		}

		// 解析浮点数
		char *end;
		params[param_count] = strtof(ptr, &end);

		if (end == ptr) {
			// 转换失败，跳过这个字段
			while (*end != ',' && *end != '\0') {
				end++;
			}
		} else {
			param_count++;
		}

		// 移动到下一个参数
		ptr = end;
		if (*ptr == ',') {
			ptr++;
		} else {
			break;
		}
	}

	if (param_count == 0) {
		return;
	}

	for (size_t i = 0; i < sizeof(cmd_map) / sizeof(cmd_map[0]); i++) {
		if (strcmp(cmd, cmd_map[i].cmd_name) == 0) {
			if (param_count >= cmd_map[i].min_params) {
				float input[10];
				uint8_t copy_count = (param_count < 10) ? param_count : 10;

				for (int j = 0; j < copy_count; j++) {
					input[j] = params[j];
				}

				if (cmd_map[i].data_index == INDEX_TAR) {
					motor_set_target_pos(motor_1, input[0], input[1]);
					motor_set_test_target(motor_1, input[0], input[1]);
				} else if (cmd_map[i].data_index == INDEX_VP_PI) {
					// pmotor1_param->foc_param.d_axis.kp = input[0];
					// pmotor1_param->foc_param.d_axis.ki = input[1];
					// pmotor1_param->foc_param.q_axis.kp = input[2];
					// pmotor1_param->foc_param.q_axis.ki = input[3];
					pmotor1_param->foc_param.vel.kp = input[0];
					pmotor1_param->foc_param.vel.ki = input[1];
					pmotor1_param->foc_param.pos.kp = input[2];
					pmotor1_param->foc_param.pos.ki = input[3];
				} else if (cmd_map[i].data_index == INDEX_STATE) {
					// 处理状态命令
					if (input[0] == 0) {
						// 进入空闲状态
						motor_tran_state(motor_1, MOTOR_STATUS_IDLE);
					} else if (input[0] == 1) {
						// 进入运行状态
						motor_tran_state(motor_1, MOTOR_STATUS_RUNING);
					}
				} else if (cmd_map[i].data_index == INDEX_MODE) {
					// 处理模式命令
					if (input[0] > 0.5f && input[0] < 1.5f) {
						// 切换到pp模式
						motor_tran_mode(motor_1, MODE_PP);
					} else if (input[0] > 2.5f && input[0] < 3.5f) {
						// 切换到pv模式
						motor_tran_mode(motor_1, MODE_PV);
					} else if (input[0] > 8.5f && input[0] < 9.5f) {
						motor_tran_mode(motor_1, MODE_DEBUG);
					} else if (input[0] > 7.5f && input[0] < 8.5f) {
						motor_tran_mode(motor_1, MODE_DEBUG_POSVEL);
					} else {
						// 切换到无模式
						motor_tran_mode(motor_1, MODE_NONE);
					}
				}
				return;
			}
		}
	}
}
/*============================================================================
 * CANopen 定时中断处理
 *===========================================================================*/

#include "canopen_app/canopen_app.h"

/* 外部声明 main.c 中定义的 CANopen 应用实例 */
extern struct canopen_app canopen_app;

/* 定时器中断周期（微秒）- 与 TIM6 配置对应 */
#define CANOPEN_TIM_PERIOD_US 1000 /* 1ms = 1000us */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6) {
		canopen_app_interrupt(&canopen_app, CANOPEN_TIM_PERIOD_US);
	}
}
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
		// raw_uvw[2] = (uint32_t)(hadc2.Instance->JDR1);
		raw_uvw[4] = (uint32_t)(hadc->Instance->DR) + 300; // 硬件补偿，临时方案
		raw_uvw[3] = 0;
		motor_highfreq_task(motor_1, (uint16_t *)raw_uvw);
	}
}
