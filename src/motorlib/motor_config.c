#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "statemachine.h"
/* 电机1反馈实例 */
static struct feedback m1_feedback;

/* 电机1逆变器实例 */
static struct inverter m1_inverter;

/* 电机1配置 */
static struct motor_config m1_config;
/* 电机1数据 */
static struct motor_data m1_data;

static struct statemachine m1_sm;
/* 电机1实例 */
static struct motor motor1 = {
	.feedback = &m1_feedback,
	.inverter = &m1_inverter,
	.sm = &m1_sm,
	.config = &m1_config,
	.data = &m1_data,
};

/* 电机1全局指针 */
struct motor *motor_1 = &motor1;
