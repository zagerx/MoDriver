
#include "cia402.h"
#include <stdbool.h>
#include "cia402_state.h"
#undef NULL
#define NULL (0)

void cia402_params_bind(struct cia402_instance *instance, struct motor *motor,
			uint16_t *controlword, uint16_t *statusword, int8_t *modes_of_operation,
			int8_t *mode_display, int32_t *target_velocity, int32_t *actual_velocity,
			uint16_t *error_code, int32_t *target_position, int16_t *target_torque,
			int32_t *actual_position, int16_t *actual_torque)
{
	if (!instance || !motor) {
		return;
	}

	/* 检查必要参数 */
	if (!controlword || !statusword || !modes_of_operation || !mode_display ||
	    !target_velocity || !actual_velocity) {
		return;
	}

	/* 绑定 motor 实例 */
	instance->motor = motor;

	/* 绑定 OD 指针 */
	instance->controlword = controlword;
	instance->statusword = statusword;
	instance->modes_of_operation = modes_of_operation;
	instance->mode_display = mode_display;
	instance->target_velocity = target_velocity;
	instance->actual_velocity = actual_velocity;

	/* 可选参数（可为 NULL）*/
	instance->error_code = error_code;
	instance->target_position = target_position;
	instance->target_torque = target_torque;
	instance->actual_position = actual_position;
	instance->actual_torque = actual_torque;
}

void cia402_init(struct cia402_instance *instance)
{
	if (!instance || !instance->is_initialized) {
		return;
	}
	/* 初始化配置 */
	instance->supported_modes =
		(1U << 0) | (1U << 3) | (1U << 6) | (1U << 8) | (1U << 9) | (1U << 10);
	instance->quick_stop_decel = 0;

	/* 初始化状态 */
	instance->is_initialized = true;
	instance->halt_active = false;
	instance->fault_code = 0;
	/* 初始化 PDS 状态机 */
	statemachine_init(&instance->pds_sm, instance, cia402_pds_not_ready_state, NULL, 0);
}

void cia402_update(struct cia402_instance *instance, float dt)
{
	(void)dt; /* 当前版本未使用 dt 参数，保留供将来使用 */
	if (!instance || !instance->is_initialized) {
		return;
	}
	fault_handler(instance); /* 检查故障并可能触发状态转换 */

	if (*instance->modes_of_operation == CIA402_MODE_PROFILE_POSITION) {
		/* 根据当前模式转换 motor 状态 */
		motor_tran_pp_mode(instance->motor);
	} else if (*instance->modes_of_operation == CIA402_MODE_PROFILE_VELOCITY) {
		motor_tran_pv_mode(instance->motor);
	} else {
		motor_tran_none_mode(instance->motor);
	}
	sm_dispatch(&instance->pds_sm); /* 先处理状态机事件 */
}
