#ifndef CIA402_STATE_H
#define CIA402_STATE_H

#include <stdint.h>
#include "cia402_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct statemachine;
struct cia402_instance;

/*================== PDS 状态函数声明 ==================*/

/**
 * @brief Not Ready to Switch On 状态
 * 
 * 初始状态，驱动器正在初始化
 * 状态字: CIA402_STATE_NOT_READY_TO_SWITCH_ON (0x0000)
 * 
 * 状态转换:
 * - Trans 0/1: 初始化完成 -> Switch On Disabled
 */
void cia402_pds_not_ready_state(struct statemachine *sm);

/**
 * @brief Switch On Disabled 状态
 * 
 * 驱动器已初始化，但未准备好使能
 * 状态字: CIA402_STATE_SWITCH_ON_DISABLED (0x0040)
 * 
 * 有效命令:
 * - Shutdown (CIA402_CMD_SHUTDOWN / 0x06) -> Ready to Switch On (Trans 2)
 */
void cia402_pds_switch_on_disabled_state(struct statemachine *sm);

/**
 * @brief Ready to Switch On 状态
 * 
 * 驱动器已准备好使能
 * 状态字: CIA402_STATE_READY_TO_SWITCH_ON (0x0021)
 * 
 * 有效命令:
 * - Switch On (CIA402_CMD_SWITCH_ON / 0x07) -> Switched On (Trans 3)
 * - Disable Voltage (CIA402_CMD_DISABLE_VOLTAGE / 0x00) -> Switch On Disabled (Trans 7)
 * - Quick Stop (CIA402_CMD_QUICK_STOP / 0x02) -> Switch On Disabled (Trans 7)
 */
void cia402_pds_ready_to_switch_on_state(struct statemachine *sm);

/**
 * @brief Switched On 状态
 * 
 * 驱动器电源已开启，但输出未使能
 * 状态字: CIA402_STATE_SWITCHED_ON (0x0023)
 * 
 * 有效命令:
 * - Enable Operation (CIA402_CMD_ENABLE_OPERATION / 0x0F) -> Operation Enabled (Trans 4)
 * - Shutdown (CIA402_CMD_SHUTDOWN / 0x06) -> Ready to Switch On (Trans 6)
 * - Disable Voltage (CIA402_CMD_DISABLE_VOLTAGE / 0x00) -> Switch On Disabled (Trans 10)
 * - Quick Stop (CIA402_CMD_QUICK_STOP / 0x02) -> Switch On Disabled (Trans 10)
 */
void cia402_pds_switched_on_state(struct statemachine *sm);

/**
 * @brief Operation Enabled 状态
 * 
 * 驱动器完全使能，可以执行运动
 * 状态字: CIA402_STATE_OPERATION_ENABLED (0x0027)
 * 
 * 有效命令:
 * - Disable Operation (CIA402_CMD_DISABLE_OPERATION / 0x07, bit3=0) -> Switched On (Trans 5)
 * - Shutdown (CIA402_CMD_SHUTDOWN / 0x06) -> Ready to Switch On (Trans 8)
 * - Disable Voltage (CIA402_CMD_DISABLE_VOLTAGE / 0x00) -> Switch On Disabled (Trans 9)
 * - Quick Stop (CIA402_CMD_QUICK_STOP / 0x02) -> Quick Stop Active (Trans 11)
 */
void cia402_pds_operation_enabled_state(struct statemachine *sm);

/**
 * @brief Quick Stop Active 状态
 * 
 * 快速停止正在执行
 * 状态字: CIA402_STATE_QUICK_STOP_ACTIVE (0x0007)
 * 注意: Quick Stop位(CIA402_SW_QUICK_STOP_MASK / bit5)在此状态为0
 * 
 * 有效命令:
 * - Quick Stop 完成 -> Switch On Disabled (Trans 12)
 * - Enable Operation (CIA402_CMD_ENABLE_OPERATION / 0x0F) -> Operation Enabled (Trans 16, 可选)
 */
void cia402_pds_quick_stop_active_state(struct statemachine *sm);

/**
 * @brief Fault Reaction Active 状态
 * 
 * 故障反应正在执行，电机正在安全停机
 * 状态字: CIA402_STATE_FAULT_REACTION_ACTIVE (0x000F)
 * 
 * 此状态自动执行，完成后进入 Fault 状态 (Trans 14)
 */
void cia402_pds_fault_reaction_state(struct statemachine *sm);

/**
 * @brief Fault 状态
 * 
 * 故障状态，驱动器已停止
 * 状态字: CIA402_STATE_FAULT (0x0008)
 * 
 * 有效命令:
 * - Fault Reset (CIA402_CMD_FAULT_RESET / 0x80) -> Switch On Disabled (Trans 15)
 */
void cia402_pds_fault_state(struct statemachine *sm);

/*================== 控制字命令枚举 ==================*/

/**
 * @brief CIA 402 控制字命令类型
 * 
 * 对应控制字(0x6040)的不同命令组合
 */
enum cia402_command {
	CMD_NONE,                   /*!< 无有效命令 */
	CMD_SHUTDOWN_REQ,           /*!< Shutdown (0x06): bit2=1, bit1=1, bit0=0 */
	CMD_SWITCH_ON_REQ,          /*!< Switch On (0x07): bit2=1, bit1=1, bit0=1, bit3=0 */
	CMD_DISABLE_VOLTAGE_REQ,    /*!< Disable Voltage (0x00): bit1=0 */
	CMD_ENABLE_OP_REQ,          /*!< Enable Operation (0x0F): bit3=1, bit2=1, bit1=1, bit0=1 */
	CMD_DISABLE_OP_REQ,         /*!< Disable Operation (0x07): bit3=0, bit2=1, bit1=1, bit0=1 */
	CMD_QUICK_STOP_REQ,         /*!< Quick Stop (0x02): bit2=0, bit1=1 */
	CMD_FAULT_RESET_REQ,        /*!< Fault Reset (0x80): bit7=1 */
};

/*================== 内联辅助函数 ==================*/

/**
 * @brief 从状态字获取当前PDS状态
 * @param statusword 状态字值
 * @return PDS状态值 (CIA402_STATE_xxx)
 */
static inline uint16_t cia402_get_state_from_sw(uint16_t statusword)
{
	return CIA402_GET_STATE_FROM_SW(statusword);
}

/**
 * @brief 检查状态字是否表示特定状态
 * @param statusword 状态字值
 * @param state 目标状态 (CIA402_STATE_xxx)
 * @return true 如果是该状态
 */
static inline int cia402_is_state(uint16_t statusword, uint16_t state)
{
	return CIA402_IS_STATE(statusword, state);
}

/**
 * @brief 检查当前是否为 Operation Enabled 状态
 * @param statusword 状态字值
 * @return true 如果是 Operation Enabled 状态
 */
static inline int cia402_is_operation_enabled(uint16_t statusword)
{
	return cia402_is_state(statusword, CIA402_STATE_OPERATION_ENABLED);
}

/**
 * @brief 检查当前是否为 Fault 状态
 * @param statusword 状态字值
 * @return true 如果是 Fault 状态
 */
static inline int cia402_is_fault(uint16_t statusword)
{
	return (statusword & CIA402_SW_FAULT_MASK) != 0;
}

/**
 * @brief 构建控制字命令
 * @param so Switch On (bit0)
 * @param ev Enable Voltage (bit1)
 * @param qs Quick Stop (bit2)
 * @param eo Enable Operation (bit3)
 * @param fr Fault Reset (bit7)
 * @return 控制字命令值
 */
static inline uint16_t cia402_build_cw_cmd(int so, int ev, int qs, int eo, int fr)
{
	return CIA402_BUILD_CW_CMD(so, ev, qs, eo, fr);
}

/**
 * @brief 获取标准控制字命令值
 * @param cmd 命令类型
 * @return 控制字值
 */
static inline uint16_t cia402_get_std_cw_cmd(enum cia402_command cmd)
{
	switch (cmd) {
	case CMD_SHUTDOWN_REQ:
		return CIA402_CMD_SHUTDOWN;
	case CMD_SWITCH_ON_REQ:
		return CIA402_CMD_SWITCH_ON;
	case CMD_DISABLE_VOLTAGE_REQ:
		return CIA402_CMD_DISABLE_VOLTAGE;
	case CMD_ENABLE_OP_REQ:
		return CIA402_CMD_ENABLE_OPERATION;
	case CMD_DISABLE_OP_REQ:
		return CIA402_CMD_DISABLE_OPERATION;
	case CMD_QUICK_STOP_REQ:
		return CIA402_CMD_QUICK_STOP;
	case CMD_FAULT_RESET_REQ:
		return CIA402_CMD_FAULT_RESET;
	default:
		return 0;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* CIA402_STATE_H */
