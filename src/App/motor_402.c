
#include "motor_402.h"
#include <stdbool.h>
#undef NULL
#define NULL (0)

static void cia402_pds_not_ready_state(struct statemachine *sm);

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
static void cia402_pds_not_ready_state(struct statemachine *sm)
{
	enum {
		CHECK_READY = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		/* 初始化状态：状态字 bit9=0, bit8=0, bit6=0, bit5=0, bit3=0, bit2=0, bit1=0, bit0=0
		 */
		if (inst && inst->statusword) {
			*inst->statusword &= ~0x014F; /* 清除相关位 */
		}
		sm->phase = CHECK_READY;
		break;

	case CHECK_READY:
		/* 检查是否完成初始化（如自检通过），然后自动切换到 Switch On Disabled */
		if (inst && inst->is_initialized) {
			// TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
		}
		break;

	case EXIT:
		/* 离开 Not Ready，进入 Switch On Disabled 时设置状态字 bit6=1, bit5=1 */
		if (inst && inst->statusword) {
			*inst->statusword = 0x0040; /* Switch On Disabled: bit6=1 */
		}
		break;

	default:
		break;
	}
}
