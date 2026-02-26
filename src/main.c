#include <stddef.h>

#include "hardware.h"
#include "motor.h"

int main(void)
{
	hardware_init();
	motor_register_callback(motor_1, encoder_getraw, NULL, NULL, NULL);

	while (1) {
	}
}
