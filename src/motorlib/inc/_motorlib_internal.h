#ifndef _MOTOR_INTERNAL_H
#define _MOTOR_INTERNAL_H

#include "inverter.h"
#include "feedback.h"

struct motor_data {
	int16_t error_code;
};

struct motor_config {
	struct inverter *inverter;
	struct feedback *feedback;
};

struct motor {
	struct motor_data *data;
	struct motor_config *config;
};

#endif
