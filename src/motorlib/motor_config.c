#include "_motorlib_internal.h"

struct feedback m1_feedback;
struct motor_config m1_config = {
	.feedback = &m1_feedback,
};

struct motor_data m1_data;

struct motor motor1 = {
	.config = &m1_config,
	.data = &m1_data,
};

struct motor *motor_1 = &motor1;
