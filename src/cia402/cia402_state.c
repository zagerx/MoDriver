#include "cia402.h"
#include "cia402_state.h"
#include "cia402_defs.h"

/*============================================================================
 * 辅助函数
 *===========================================================================*/

/**
 * @brief 设置状态字
 */
static inline void set_statusword(struct cia402_instance *inst, uint16_t value)
{
	if (inst && inst->statusword) {
		/* 保留制造商特定位 (bit8-15) */
		uint16_t preserved = *inst->statusword & 0xFF00;
		*inst->statusword = (value & CIA402_STATE_MASK) | preserved;
	}
}

/**
 * @brief 检查是否请求 Halt (控制字 bit8)
 * @param cw 控制字值
 * @return true 如果 bit8=1 (请求暂停)
 */
static inline bool is_halt_requested(uint16_t cw)
{
	return (cw & CIA402_CW_HALT_MASK) != 0;
}

/**
 * @brief 检查是否有故障
 */
static bool has_fault(struct cia402_instance *inst)
{
	if (!inst) {
		return false;
	}
	if (inst->error_code && (*inst->error_code != 0)) {
		return true;
	}
	if (inst->fault_code != 0) {
		return true;
	}
	return false;
}

/**
 * @brief 解析控制字命令
 * 
 * 根据 CIA 402 标准解析控制字命令 (bits 0-3, 7)
 * 
 * @param cw 控制字值
 * @return 命令类型
 */
static enum cia402_command parse_controlword(uint16_t cw)
{
	/* 故障复位命令优先级最高 (bit7) */
	if (CIA402_IS_FAULT_RESET_REQ(cw)) {
		return CMD_FAULT_RESET_REQ;
	}

	/* 提取关键位 */
	uint8_t bit0 = (cw & CIA402_CW_SWITCH_ON_MASK) ? 1 : 0;
	uint8_t bit1 = (cw & CIA402_CW_ENABLE_VOLTAGE_MASK) ? 1 : 0;
	uint8_t bit2 = (cw & CIA402_CW_QUICK_STOP_MASK) ? 1 : 0;
	uint8_t bit3 = (cw & CIA402_CW_ENABLE_OPERATION_MASK) ? 1 : 0;

	/* Enable Operation: bit3=1, bit2=1, bit1=1, bit0=1 (0x0F) */
	if (bit3 && bit2 && bit1 && bit0) {
		return CMD_ENABLE_OP_REQ;
	}

	/* 
	 * Switch On / Disable Operation: bit2=1, bit1=1, bit0=1 (0x07)
	 * 注意：0x07 在不同状态下含义不同：
	 *   - 在 Ready to Switch On 状态: Switch On (Trans 3)
	 *   - 在 Operation Enabled 状态: Disable Operation (Trans 5)
	 * 这里统一返回 CMD_SWITCH_ON_REQ，由状态机根据当前状态处理
	 */
	if (bit2 && bit1 && bit0) {
		return CMD_SWITCH_ON_REQ;
	}

	/* Shutdown: bit2=1, bit1=1, bit0=0 (0x06) */
	if (bit2 && bit1 && !bit0) {
		return CMD_SHUTDOWN_REQ;
	}

	/* Quick Stop: bit2=0, bit1=1 (0x02, 0x03) */
	if (!bit2 && bit1) {
		return CMD_QUICK_STOP_REQ;
	}

	/* Disable Voltage: bit1=0 (0x00, 0x01, 0x04, 0x05) */
	if (!bit1) {
		return CMD_DISABLE_VOLTAGE_REQ;
	}

	return CMD_NONE;
}

/*============================================================================
 * PDS 状态实现
 *===========================================================================*/

/**
 * @brief Not Ready to Switch On 状态
 * 
 * 初始状态，驱动器正在初始化
 * 状态字: 0x0000 (bit6=0, bit3=0, bit2=0, bit1=0, bit0=0)
 * 
 * 转换:
 * - 自动 (Trans 0/1): 初始化完成 -> Switch On Disabled
 */
void cia402_pds_not_ready_state(struct statemachine *sm)
{
	enum {
		CHECK_READY = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		/* 清除状态字中的状态位 */
		if (inst && inst->statusword) {
			*inst->statusword &= ~CIA402_STATE_MASK;
		}
		sm->phase = CHECK_READY;
		break;

	case CHECK_READY:
		/* 检查是否完成初始化，自动切换到 Switch On Disabled */
		if (inst && inst->is_initialized) {
			TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
		}
		break;

	case EXIT:
		/* 设置 Switch On Disabled 状态字 */
		set_statusword(inst, CIA402_STATE_SWITCH_ON_DISABLED);
		break;

	default:
		break;
	}
}

/**
 * @brief Switch On Disabled 状态
 * 
 * 驱动器已初始化，但未准备好使能
 * 状态字: 0x0040 (bit6=1, Switch On Disabled)
 * 
 * 有效命令:
 * - Shutdown (0x06) -> Ready to Switch On (Trans 2)
 */
void cia402_pds_switch_on_disabled_state(struct statemachine *sm)
{
	enum {
		CHECK_COMMAND = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_SWITCH_ON_DISABLED);
		sm->phase = CHECK_COMMAND;
		break;

	case CHECK_COMMAND:
		/* 检查故障 */
		if (has_fault(inst)) {
			TRAN_STATE(sm, cia402_pds_fault_reaction_state);
			break;
		}

		/* 解析控制字命令 */
		if (inst && inst->controlword) {
			enum cia402_command cmd = parse_controlword(*inst->controlword);

			if (cmd == CMD_SHUTDOWN_REQ) {
				/* 转换 2: Switch On Disabled -> Ready to Switch On */
				TRAN_STATE(sm, cia402_pds_ready_to_switch_on_state);
			}
		}
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief Ready to Switch On 状态
 * 
 * 驱动器已准备好使能
 * 状态字: 0x0021 (bit6=0, bit5=1, bit0=1)
 * 
 * 有效命令:
 * - Switch On (0x07) -> Switched On (Trans 3)
 * - Disable Voltage (0x00) -> Switch On Disabled (Trans 7)
 * - Quick Stop (0x02) -> Switch On Disabled (Trans 7)
 */
void cia402_pds_ready_to_switch_on_state(struct statemachine *sm)
{
	enum {
		CHECK_COMMAND = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_READY_TO_SWITCH_ON);
		sm->phase = CHECK_COMMAND;
		break;

	case CHECK_COMMAND:
		/* 检查故障 */
		if (has_fault(inst)) {
			TRAN_STATE(sm, cia402_pds_fault_reaction_state);
			break;
		}

		/* 解析控制字命令 */
		if (inst && inst->controlword) {
			enum cia402_command cmd = parse_controlword(*inst->controlword);

			switch (cmd) {
			case CMD_SWITCH_ON_REQ:
				/* 转换 3: Ready to Switch On -> Switched On */
				TRAN_STATE(sm, cia402_pds_switched_on_state);
				break;

			case CMD_DISABLE_VOLTAGE_REQ:
			case CMD_QUICK_STOP_REQ:
				/* 转换 7: Ready to Switch On -> Switch On Disabled */
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
				break;

			default:
				/* Shutdown 命令在此状态保持当前状态 */
				break;
			}
		}
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief Switched On 状态
 * 
 * 驱动器电源已开启，但输出未使能
 * 状态字: 0x0023 (bit6=0, bit5=1, bit1=1, bit0=1)
 * 
 * 有效命令:
 * - Enable Operation (0x0F) -> Operation Enabled (Trans 4)
 * - Shutdown (0x06) -> Ready to Switch On (Trans 6)
 * - Disable Voltage (0x00) -> Switch On Disabled (Trans 10)
 * - Quick Stop (0x02) -> Switch On Disabled (Trans 10)
 */
void cia402_pds_switched_on_state(struct statemachine *sm)
{
	enum {
		CHECK_COMMAND = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_SWITCHED_ON);
		/* TODO: 执行电源开启操作 */
		sm->phase = CHECK_COMMAND;
		break;

	case CHECK_COMMAND:
		/* 检查故障 */
		if (has_fault(inst)) {
			TRAN_STATE(sm, cia402_pds_fault_reaction_state);
			break;
		}

		/* 解析控制字命令 */
		if (inst && inst->controlword) {
			enum cia402_command cmd = parse_controlword(*inst->controlword);

			switch (cmd) {
			case CMD_ENABLE_OP_REQ:
				/* 转换 4: Switched On -> Operation Enabled */
				TRAN_STATE(sm, cia402_pds_operation_enabled_state);
				break;

			case CMD_SHUTDOWN_REQ:
				/* 转换 6: Switched On -> Ready to Switch On */
				TRAN_STATE(sm, cia402_pds_ready_to_switch_on_state);
				break;

			case CMD_DISABLE_VOLTAGE_REQ:
			case CMD_QUICK_STOP_REQ:
				/* 转换 10: Switched On -> Switch On Disabled */
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
				break;

			default:
				break;
			}
		}
		break;

	case EXIT:
		/* TODO: 执行电源关闭准备 */
		break;

	default:
		break;
	}
}

/**
 * @brief Operation Enabled 状态
 * 
 * 驱动器完全使能，可以执行运动
 * 状态字: 0x0027 (bit6=0, bit5=1, bit2=1, bit1=1, bit0=1)
 * 
 * 有效命令:
 * - Disable Operation (0x07, bit3=0) -> Switched On (Trans 5)
 * - Shutdown (0x06) -> Ready to Switch On (Trans 8)
 * - Disable Voltage (0x00) -> Switch On Disabled (Trans 9)
 * - Quick Stop (0x02) -> Quick Stop Active (Trans 11)
 * 
 * Halt处理 (bit8):
 * - bit8=1: 暂停运动（使用减速度停止，保持PDS状态）
 * - bit8=0: 继续运动
 */
void cia402_pds_operation_enabled_state(struct statemachine *sm)
{
	enum {
		CHECK_COMMAND = USER_STATUS,
		RUN_OPERATION,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_OPERATION_ENABLED);
		inst->halt_active = false;
		/* TODO: 使能电机输出 */
		sm->phase = CHECK_COMMAND;
		break;

	case CHECK_COMMAND:
		/* 检查故障 */
		if (has_fault(inst)) {
			TRAN_STATE(sm, cia402_pds_fault_reaction_state);
			break;
		}

		/* 解析控制字命令 */
		if (inst && inst->controlword) {
			uint16_t cw = *inst->controlword;
			enum cia402_command cmd = parse_controlword(cw);

			/* 检查 Halt 请求 (bit8) */
			if (is_halt_requested(cw)) {
				inst->halt_active = true;
				/* TODO: 执行 Halt 减速停止 */
			} else {
				if (inst->halt_active) {
					inst->halt_active = false;
					/* TODO: 恢复运动 */
				}
			}

			switch (cmd) {
			case CMD_SWITCH_ON_REQ:
				/* 
				 * 在 Operation Enabled 状态，0x07 表示 Disable Operation
				 * 转换 5: Operation Enabled -> Switched On
				 */
				TRAN_STATE(sm, cia402_pds_switched_on_state);
				break;

			case CMD_SHUTDOWN_REQ:
				/* 转换 8: Operation Enabled -> Ready to Switch On */
				TRAN_STATE(sm, cia402_pds_ready_to_switch_on_state);
				break;

			case CMD_QUICK_STOP_REQ:
				/* 转换 11: Operation Enabled -> Quick Stop Active */
				TRAN_STATE(sm, cia402_pds_quick_stop_active_state);
				break;

			case CMD_DISABLE_VOLTAGE_REQ:
				/* 转换 9: Operation Enabled -> Switch On Disabled */
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
				break;

			default:
				sm->phase = RUN_OPERATION;
				break;
			}
		}
		break;

	case RUN_OPERATION:
		/* 检查故障 */
		if (has_fault(inst)) {
			TRAN_STATE(sm, cia402_pds_fault_reaction_state);
			break;
		}

		/* 持续检查控制字 */
		if (inst && inst->controlword) {
			uint16_t cw = *inst->controlword;
			enum cia402_command cmd = parse_controlword(cw);

			/* 检查 Halt 请求 */
			if (is_halt_requested(cw)) {
				if (!inst->halt_active) {
					inst->halt_active = true;
					/* TODO: 执行 Halt 减速停止 */
				}
			} else {
				if (inst->halt_active) {
					inst->halt_active = false;
					/* TODO: 恢复运动 */
				}
			}

			switch (cmd) {
			case CMD_SWITCH_ON_REQ:
				/* Disable Operation */
				TRAN_STATE(sm, cia402_pds_switched_on_state);
				break;

			case CMD_SHUTDOWN_REQ:
				TRAN_STATE(sm, cia402_pds_ready_to_switch_on_state);
				break;

			case CMD_QUICK_STOP_REQ:
				TRAN_STATE(sm, cia402_pds_quick_stop_active_state);
				break;

			case CMD_DISABLE_VOLTAGE_REQ:
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
				break;

			default:
				/* 正常运行业务逻辑 */
				break;
			}
		}
		break;

	case EXIT:
		inst->halt_active = false;
		/* TODO: 禁用电机输出 */
		break;

	default:
		break;
	}
}

/**
 * @brief Quick Stop Active 状态
 * 
 * 快速停止正在执行
 * 状态字: 0x0007 (bit6=0, bit2=1, bit1=1, bit0=1)
 * 注意: Quick Stop位(bit5)在此状态为0
 * 
 * 有效命令:
 * - Quick Stop 完成 -> Switch On Disabled (Trans 12)
 * - Enable Operation (0x0F) -> Operation Enabled (Trans 16, 可选)
 */
void cia402_pds_quick_stop_active_state(struct statemachine *sm)
{
	enum {
		EXECUTE_QUICK_STOP = USER_STATUS,
		CHECK_COMPLETE,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_QUICK_STOP_ACTIVE);
		if (inst) {
			inst->halt_active = true;
		}
		/* TODO: 启动快速停止程序 */
		sm->phase = EXECUTE_QUICK_STOP;
		break;

	case EXECUTE_QUICK_STOP:
		/* TODO: 执行快速停止减速 */
		sm->phase = CHECK_COMPLETE;
		break;

	case CHECK_COMPLETE:
		/* 检查快速停止是否完成 */
		{
			bool quick_stop_complete = false;
			/* TODO: 检查电机实际速度是否为0 */
			if (inst && inst->actual_velocity) {
				quick_stop_complete = (*inst->actual_velocity == 0);
			}

			/* 检查是否允许从 Quick Stop 恢复 */
			if (inst && inst->controlword) {
				enum cia402_command cmd = parse_controlword(*inst->controlword);

				if (cmd == CMD_ENABLE_OP_REQ) {
					/* 转换 16: Quick Stop Active -> Operation Enabled */
					/* 注意: 某些实现不允许此转换，取决于 Quick Stop Option Code */
					inst->halt_active = false;
					TRAN_STATE(sm, cia402_pds_operation_enabled_state);
					break;
				}
			}

			if (quick_stop_complete) {
				/* 转换 12: Quick Stop Active -> Switch On Disabled */
				inst->halt_active = false;
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
			}
		}
		break;

	case EXIT:
		inst->halt_active = false;
		break;

	default:
		break;
	}
}

/**
 * @brief Fault Reaction Active 状态
 * 
 * 故障反应正在执行，电机正在安全停机
 * 状态字: 0x000F (bit6=0, bit3=1, bit2=1, bit1=1, bit0=1)
 * 
 * 此状态自动执行，完成后进入 Fault 状态 (Trans 14)
 */
void cia402_pds_fault_reaction_state(struct statemachine *sm)
{
	enum {
		EXECUTE_FAULT_REACTION = USER_STATUS,
		CHECK_COMPLETE,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_FAULT_REACTION_ACTIVE);
		/* TODO: 启动故障反应程序（安全停机） */
		sm->phase = EXECUTE_FAULT_REACTION;
		break;

	case EXECUTE_FAULT_REACTION:
		/* TODO: 执行故障反应（如快速停机） */
		sm->phase = CHECK_COMPLETE;
		break;

	case CHECK_COMPLETE:
		/* 故障反应完成后自动进入 Fault 状态 */
		TRAN_STATE(sm, cia402_pds_fault_state);
		break;

	case EXIT:
		break;

	default:
		break;
	}
}

/**
 * @brief Fault 状态
 * 
 * 故障状态，驱动器已停止
 * 状态字: 0x0008 (bit6=0, bit3=1)
 * 
 * 有效命令:
 * - Fault Reset (0x80) -> Switch On Disabled (Trans 15)
 */
void cia402_pds_fault_state(struct statemachine *sm)
{
	enum {
		WAIT_FAULT_RESET = USER_STATUS,
	};

	struct cia402_instance *inst = (struct cia402_instance *)(sm->data);

	switch (sm->phase) {
	case ENTER:
		set_statusword(inst, CIA402_STATE_FAULT);
		/* TODO: 确保电机已禁用 */
		sm->phase = WAIT_FAULT_RESET;
		break;

	case WAIT_FAULT_RESET:
		/* 解析控制字命令 */
		if (inst && inst->controlword) {
			enum cia402_command cmd = parse_controlword(*inst->controlword);

			if (cmd == CMD_FAULT_RESET_REQ) {
				/* 转换 15: Fault -> Switch On Disabled */
				/* 清除故障标志 */
				if (inst->error_code) {
					*inst->error_code = 0;
				}
				inst->fault_code = 0;
				TRAN_STATE(sm, cia402_pds_switch_on_disabled_state);
			}
		}
		break;

	case EXIT:
		/* 故障已清除，准备恢复正常操作 */
		break;

	default:
		break;
	}
}
