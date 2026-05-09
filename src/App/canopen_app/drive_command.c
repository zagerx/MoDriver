#include "drive_command.h"
#include "motor.h"
#include "motor_interface_bits.h"
#include "motor_interface_mode.h"
#include "301/CO_ODinterface.h"
#include "OD.h"
#include <string.h>

void drive_command_params_bind(struct drive_command_instance *inst, struct motor *motor,
			       uint8_t *cmd_id, float *arg1)
{
	if (!inst || !motor || !cmd_id || !arg1) {
		return;
	}

	inst->motor = motor;
	inst->cmd_id = cmd_id;
	inst->arg1 = arg1;
}

void drive_command_init(struct drive_command_instance *inst)
{
	if (!inst) {
		return;
	}

	inst->cache_cmd = 0;
}

void drive_command_update(struct drive_command_instance *inst)
{
	if (!inst || !inst->motor) {
		return;
	}

	uint8_t cmd = *inst->cmd_id;
	if (cmd == 0) {
		return;
	}
	inst->cache_cmd = cmd;
	*inst->cmd_id = 0; /* 执行后清零指令码，避免重复执行 */
	float arg1 = *inst->arg1;

	switch (cmd) {
	case DCMD_MOTOR_TRAN_RUNING:
		motor_tran_state(inst->motor, MOTOR_STATUS_RUNNING);
		break;

	case DCMD_MOTOR_TARN_IDLE:
		motor_tran_state(inst->motor, MOTOR_STATUS_IDLE);
		break;

	case DCMD_MOTOR_TARN_CALIB:
		motor_tran_state(inst->motor, MOTOR_STATUS_CALIB);
		break;

	case DCMD_MOTOR_TARN_MODE_DEBUG:
		motor_tran_mode(inst->motor, MODE_DEBUG);
		break;

	case DCMD_MOTOR_RAN_MODE_P:
		motor_tran_mode(inst->motor, MODE_P);
		break;

	case DCMD_MOTOR_RAN_MODE_V:
		motor_tran_mode(inst->motor, MODE_V);
		break;

	case DCMD_SET_VELEOIVER_TARVAL:
		/* TODO: 速度目标值设定 */
		motor_set_test_target(inst->motor, arg1, 0.0f);
		break;

	case DCMD_SET_POSITON_TARVAL:
		motor_set_test_target(inst->motor, 0.0f, arg1);
		break;

	default:
		break;
	}
}
