#ifndef _MOTOR_INTERNAL_H
#define _MOTOR_INTERNAL_H

#include "motor_driver.h"
struct statemachine;
struct inverter;
struct feedback;

struct motor_data {
	int16_t error_code;
};

struct motor_config {
	uint16_t pairs;
};

struct motor {
	struct inverter *inverter;
	struct feedback *feedback;
	struct statemachine *sm;
	struct motor_data *data;
	struct motor_config *config;
};

#endif
