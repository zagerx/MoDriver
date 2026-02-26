#include "gpio.h"

void as5047_init(void)
{
	HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
}
uint16_t encoder_getraw(void)
{
	return 0;
}
