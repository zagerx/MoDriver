#include "motor.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
extern uint8_t sg_uartreceive_buff[125];
extern struct motor_param_ext m1_param_ext;
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
};

// 命令映射表结构
typedef struct {
	const char *cmd_name;
	uint8_t min_params; // 最少需要的参数个数
	enum foc_data_index data_index;
} command_map_t;

// 命令表定义
static const command_map_t cmd_map[] = {
	{"tar", 1, INDEX_TAR},
	{"pid", 6, INDEX_VP_PI},
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
					m1_param_ext.foc_param.target_pos = input[0];
					m1_param_ext.foc_param.target_vel = input[0];
				} else if (cmd_map[i].data_index == INDEX_VP_PI) {

					// m1_param_ext.foc_param.d_axis.kp = input[0];
					// m1_param_ext.foc_param.d_axis.ki = input[1];
					// m1_param_ext.foc_param.q_axis.kp = input[2];
					// m1_param_ext.foc_param.q_axis.ki = input[3];
					m1_param_ext.foc_param.vel.kp = input[4];
					m1_param_ext.foc_param.vel.ki = input[5];
					m1_param_ext.foc_param.pos.kp = input[6];
					m1_param_ext.foc_param.pos.ki = input[7];
				}
			}
			return;
		}
	}
}
extern void canopen_app_interrupt(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6) {
		canopen_app_interrupt();
	}
}
