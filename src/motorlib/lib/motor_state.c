#include "statemachine.h"

/**
 * motor_carib_state - 校准状态
 * @sm: 状态机实例
 *
 * 电机参数异常时进入的校准状态，用于参数标定
 */
void motor_carib_state(struct statemachine *sm)
{
	enum {
		CALIBRATING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	switch (sm->phase) {
	case ENTER:
		sm->phase = CALIBRATING;
		break;
	case CALIBRATING:
		break;
	case EXIT:
		break;
	default:
		break;
	}
}

/**
 * motor_idle_state - 空闲状态
 * @sm: 状态机实例
 *
 * 电机初始化后的默认状态，等待运行指令
 */
void motor_idle_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	struct motor *motor = (struct motor *)(sm->data);
	(void)motor;
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}
}

/**
 * motor_runing_state - 运行状态
 * @sm: 状态机实例
 *
 * 电机正常运行状态，执行FOC控制循环
 */
void motor_runing_state(struct statemachine *sm)
{
	enum {
		RUNING = USER_STATUS,
	};
	switch (sm->phase) {
	case ENTER:
		sm->phase = RUNING;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}
}
