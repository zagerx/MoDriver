#ifndef CIA402_DEFS_H
#define CIA402_DEFS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * CIA 402 对象字典索引定义
 *===========================================================================*/

/* 控制与状态对象 */
#define CIA402_OD_CONTROLWORD                0x6040 /*!< 控制字 */
#define CIA402_OD_STATUSWORD                 0x6041 /*!< 状态字 */
#define CIA402_OD_MODES_OF_OPERATION         0x6060 /*!< 操作模式 */
#define CIA402_OD_MODES_OF_OPERATION_DISPLAY 0x6061 /*!< 操作模式显示 */
#define CIA402_OD_ERROR_CODE                 0x603F /*!< 错误码 */

/* 位置控制对象 */
#define CIA402_OD_TARGET_POSITION 0x607A /*!< 目标位置 */
#define CIA402_OD_ACTUAL_POSITION 0x6064 /*!< 实际位置 */
#define CIA402_OD_POSITION_DEMAND 0x6062 /*!< 位置需求值 */

/* 速度控制对象 */
#define CIA402_OD_TARGET_VELOCITY 0x60FF /*!< 目标速度 */
#define CIA402_OD_ACTUAL_VELOCITY 0x606C /*!< 实际速度 */
#define CIA402_OD_VELOCITY_DEMAND 0x606B /*!< 速度需求值 */

/* 转矩控制对象 */
#define CIA402_OD_TARGET_TORQUE 0x6071 /*!< 目标转矩 */
#define CIA402_OD_ACTUAL_TORQUE 0x6077 /*!< 实际转矩 */
#define CIA402_OD_TORQUE_DEMAND 0x6074 /*!< 转矩需求值 */

/* 配置对象 */
#define CIA402_OD_SUPPORTED_DRIVE_MODES 0x6502 /*!< 支持的驱动模式 */

/*============================================================================
 * 控制字 (Controlword - 0x6040) 位定义
 *===========================================================================*/

/* 位 0: Switch On (SO) */
#define CIA402_CW_SWITCH_ON_BIT  0
#define CIA402_CW_SWITCH_ON_MASK (1U << CIA402_CW_SWITCH_ON_BIT)

/* 位 1: Enable Voltage (EV) */
#define CIA402_CW_ENABLE_VOLTAGE_BIT  1
#define CIA402_CW_ENABLE_VOLTAGE_MASK (1U << CIA402_CW_ENABLE_VOLTAGE_BIT)

/* 位 2: Quick Stop (QS) - 0表示激活 */
#define CIA402_CW_QUICK_STOP_BIT  2
#define CIA402_CW_QUICK_STOP_MASK (1U << CIA402_CW_QUICK_STOP_BIT)

/* 位 3: Enable Operation (EO) */
#define CIA402_CW_ENABLE_OPERATION_BIT  3
#define CIA402_CW_ENABLE_OPERATION_MASK (1U << CIA402_CW_ENABLE_OPERATION_BIT)

/* 位 4-6: Operation Mode Specific (OMS) */
#define CIA402_CW_OMS_BITS_START 4
#define CIA402_CW_OMS_MASK       (0x07U << CIA402_CW_OMS_BITS_START)

/* 位 4: New Set-point (PP模式) / Homing Operation Start (HM模式) */
#define CIA402_CW_NEW_SET_POINT_BIT  4
#define CIA402_CW_NEW_SET_POINT_MASK (1U << CIA402_CW_NEW_SET_POINT_BIT)

/* 位 5: Change Set Immediately (PP模式) */
#define CIA402_CW_CHANGE_SET_IMM_BIT  5
#define CIA402_CW_CHANGE_SET_IMM_MASK (1U << CIA402_CW_CHANGE_SET_IMM_BIT)

/* 位 6: Absolute/Relative (PP模式) */
#define CIA402_CW_ABS_REL_BIT  6
#define CIA402_CW_ABS_REL_MASK (1U << CIA402_CW_ABS_REL_BIT)

/* 位 7: Fault Reset (F) */
#define CIA402_CW_FAULT_RESET_BIT  7
#define CIA402_CW_FAULT_RESET_MASK (1U << CIA402_CW_FAULT_RESET_BIT)

/* 位 8: Halt (H) */
#define CIA402_CW_HALT_BIT  8
#define CIA402_CW_HALT_MASK (1U << CIA402_CW_HALT_BIT)

/* 位 9: Operation Mode Specific (OMS) */
#define CIA402_CW_OMS_BIT9_BIT  9
#define CIA402_CW_OMS_BIT9_MASK (1U << CIA402_CW_OMS_BIT9_BIT)

/* 位 10: Reserved */
#define CIA402_CW_RESERVED_BIT10_BIT  10
#define CIA402_CW_RESERVED_BIT10_MASK (1U << CIA402_CW_RESERVED_BIT10_BIT)

/* 位 11-15: Manufacturer Specific (MS) */
#define CIA402_CW_MFR_SPECIFIC_BITS_START 11
#define CIA402_CW_MFR_SPECIFIC_MASK       (0x1FU << CIA402_CW_MFR_SPECIFIC_BITS_START)

/*============================================================================
 * 控制字命令组合 (Controlword Commands)
 *
 * 根据 CIA 402 标准，状态转换由控制字的位 0-3 和位 7 控制
 *===========================================================================*/

/* 基础命令掩码 (bits 0-3, 7) */
#define CIA402_CW_CMD_MASK 0x008FU

/* Shutdown: bit2=1, bit1=1, bit0=0 (0x06) */
#define CIA402_CMD_SHUTDOWN 0x0006U

/* Switch On: bit2=1, bit1=1, bit0=1 (0x07) */
#define CIA402_CMD_SWITCH_ON 0x0007U

/* Disable Voltage: bit1=0 (0x00) */
#define CIA402_CMD_DISABLE_VOLTAGE 0x0000U

/* Quick Stop: bit2=0, bit1=1 (0x02) */
#define CIA402_CMD_QUICK_STOP 0x0002U

/* Disable Operation: bit3=0, bit2=1, bit1=1, bit0=1 (0x07) */
#define CIA402_CMD_DISABLE_OPERATION 0x0007U

/* Enable Operation: bit3=1, bit2=1, bit1=1, bit0=1 (0x0F) */
#define CIA402_CMD_ENABLE_OPERATION 0x000FU

/* Fault Reset: bit7=1 (0x80) */
#define CIA402_CMD_FAULT_RESET 0x0080U

/*============================================================================
 * 状态字 (Statusword - 0x6041) 位定义
 *===========================================================================*/

/* 位 0: Ready to Switch On (RTSO) */
#define CIA402_SW_READY_TO_SWITCH_ON_BIT  0
#define CIA402_SW_READY_TO_SWITCH_ON_MASK (1U << CIA402_SW_READY_TO_SWITCH_ON_BIT)

/* 位 1: Switched On (SO) */
#define CIA402_SW_SWITCHED_ON_BIT  1
#define CIA402_SW_SWITCHED_ON_MASK (1U << CIA402_SW_SWITCHED_ON_BIT)

/* 位 2: Operation Enabled (OE) */
#define CIA402_SW_OPERATION_ENABLED_BIT  2
#define CIA402_SW_OPERATION_ENABLED_MASK (1U << CIA402_SW_OPERATION_ENABLED_BIT)

/* 位 3: Fault (F) */
#define CIA402_SW_FAULT_BIT  3
#define CIA402_SW_FAULT_MASK (1U << CIA402_SW_FAULT_BIT)

/* 位 4: Voltage Enabled (VE) */
#define CIA402_SW_VOLTAGE_ENABLED_BIT  4
#define CIA402_SW_VOLTAGE_ENABLED_MASK (1U << CIA402_SW_VOLTAGE_ENABLED_BIT)

/* 位 5: Quick Stop (QS) - 0表示Quick Stop激活 */
#define CIA402_SW_QUICK_STOP_BIT  5
#define CIA402_SW_QUICK_STOP_MASK (1U << CIA402_SW_QUICK_STOP_BIT)

/* 位 6: Switch On Disabled (SOD) */
#define CIA402_SW_SWITCH_ON_DISABLED_BIT  6
#define CIA402_SW_SWITCH_ON_DISABLED_MASK (1U << CIA402_SW_SWITCH_ON_DISABLED_BIT)

/* 位 7: Warning (W) */
#define CIA402_SW_WARNING_BIT  7
#define CIA402_SW_WARNING_MASK (1U << CIA402_SW_WARNING_BIT)

/* 位 8: Manufacturer Specific (MS) */
#define CIA402_SW_MFR_SPECIFIC_BIT8_BIT  8
#define CIA402_SW_MFR_SPECIFIC_BIT8_MASK (1U << CIA402_SW_MFR_SPECIFIC_BIT8_BIT)

/* 位 9: Remote (R) */
#define CIA402_SW_REMOTE_BIT  9
#define CIA402_SW_REMOTE_MASK (1U << CIA402_SW_REMOTE_BIT)

/* 位 10: Target Reached (TR) */
#define CIA402_SW_TARGET_REACHED_BIT  10
#define CIA402_SW_TARGET_REACHED_MASK (1U << CIA402_SW_TARGET_REACHED_BIT)

/* 位 11: Internal Limit Active (ILA) */
#define CIA402_SW_INTERNAL_LIMIT_ACTIVE_BIT  11
#define CIA402_SW_INTERNAL_LIMIT_ACTIVE_MASK (1U << CIA402_SW_INTERNAL_LIMIT_ACTIVE_BIT)

/* 位 12: Operation Mode Specific (OMS) */
#define CIA402_SW_OMS_BIT12_BIT  12
#define CIA402_SW_OMS_BIT12_MASK (1U << CIA402_SW_OMS_BIT12_BIT)

/* 位 13: Operation Mode Specific (OMS) */
#define CIA402_SW_OMS_BIT13_BIT  13
#define CIA402_SW_OMS_BIT13_MASK (1U << CIA402_SW_OMS_BIT13_BIT)

/* 位 14: Manufacturer Specific (MS) */
#define CIA402_SW_MFR_SPECIFIC_BIT14_BIT  14
#define CIA402_SW_MFR_SPECIFIC_BIT14_MASK (1U << CIA402_SW_MFR_SPECIFIC_BIT14_BIT)

/* 位 15: Manufacturer Specific (MS) */
#define CIA402_SW_MFR_SPECIFIC_BIT15_BIT  15
#define CIA402_SW_MFR_SPECIFIC_BIT15_MASK (1U << CIA402_SW_MFR_SPECIFIC_BIT15_BIT)

/*============================================================================
 * 状态字状态值 (Statusword State Values)
 *
 * 根据 CIA 402 标准，状态由位 0-3, 5, 6 的组合决定
 *===========================================================================*/

/* Not Ready to Switch On: bit6=0, bit3=0, bit2=0, bit1=0, bit0=0 (0x0000) */
#define CIA402_STATE_NOT_READY_TO_SWITCH_ON 0x0000U

/* Switch On Disabled: bit6=1 (0x0040) */
#define CIA402_STATE_SWITCH_ON_DISABLED 0x0040U

/* Ready to Switch On: bit6=0, bit5=1, bit0=1 (0x0021) */
#define CIA402_STATE_READY_TO_SWITCH_ON 0x0021U

/* Switched On: bit6=0, bit5=1, bit1=1, bit0=1 (0x0023) */
#define CIA402_STATE_SWITCHED_ON 0x0023U

/* Operation Enabled: bit6=0, bit5=1, bit2=1, bit1=1, bit0=1 (0x0027) */
#define CIA402_STATE_OPERATION_ENABLED 0x0027U

/* Quick Stop Active: bit6=0, bit2=1, bit1=1, bit0=1 (0x0007) */
/* 注意: Quick Stop位(bit5)在此状态为0 */
#define CIA402_STATE_QUICK_STOP_ACTIVE 0x0007U

/* Fault Reaction Active: bit6=0, bit3=1, bit2=1, bit1=1, bit0=1 (0x000F) */
#define CIA402_STATE_FAULT_REACTION_ACTIVE 0x000FU

/* Fault: bit6=0, bit3=1 (0x0008) */
#define CIA402_STATE_FAULT 0x0008U

/* 状态掩码 (用于提取状态位) */
#define CIA402_STATE_MASK 0x006FU

/*============================================================================
 * 操作模式 (Modes of Operation - 0x6060)
 *===========================================================================*/

#define CIA402_MODE_NONE                    0   /*!< 无模式 */
#define CIA402_MODE_PROFILE_POSITION        1   /*!< 轮廓位置模式 (PP) */
#define CIA402_MODE_PROFILE_VELOCITY        3   /*!< 轮廓速度模式 (PV) */
#define CIA402_MODE_PROFILE_TORQUE          4   /*!< 轮廓转矩模式 (PT) */
#define CIA402_MODE_HOMING                  6   /*!< 原点回归模式 (HM) */
#define CIA402_MODE_INTERPOLATED_POSITION   7   /*!< 插补位置模式 (IP) */
#define CIA402_MODE_CYCLIC_SYNC_POSITION    8   /*!< 循环同步位置模式 (CSP) */
#define CIA402_MODE_CYCLIC_SYNC_VELOCITY    9   /*!< 循环同步速度模式 (CSV) */
#define CIA402_MODE_CYCLIC_SYNC_TORQUE      10  /*!< 循环同步转矩模式 (CST) */

/*============================================================================
 * 辅助宏函数
 *===========================================================================*/

/**
 * @brief 从状态字提取PDS状态
 */
#define CIA402_GET_STATE_FROM_SW(statusword) ((statusword) & CIA402_STATE_MASK)

/**
 * @brief 检查当前是否处于特定状态
 */
#define CIA402_IS_STATE(statusword, state) (CIA402_GET_STATE_FROM_SW(statusword) == (state))

/**
 * @brief 提取控制字命令 (bits 0-3, 7)
 */
#define CIA402_GET_CMD_FROM_CW(controlword) ((controlword) & CIA402_CW_CMD_MASK)

/**
 * @brief 检查控制字中是否设置了特定命令
 */
#define CIA402_CW_HAS_CMD(controlword, cmd) (CIA402_GET_CMD_FROM_CW(controlword) == (cmd))

/**
 * @brief 检查是否请求故障复位
 */
#define CIA402_IS_FAULT_RESET_REQ(controlword) (((controlword) & CIA402_CW_FAULT_RESET_MASK) != 0)

/**
 * @brief 检查是否请求Quick Stop (bit2=0, bit1=1)
 */
#define CIA402_IS_QUICK_STOP_REQ(controlword)                                                      \
	((((controlword) & CIA402_CW_QUICK_STOP_MASK) == 0) &&                                     \
	 (((controlword) & CIA402_CW_ENABLE_VOLTAGE_MASK) != 0))

/**
 * @brief 构建控制字命令
 */
#define CIA402_BUILD_CW_CMD(so, ev, qs, eo, fr)                                                    \
	(((so) ? CIA402_CW_SWITCH_ON_MASK : 0) | ((ev) ? CIA402_CW_ENABLE_VOLTAGE_MASK : 0) |      \
	 ((qs) ? CIA402_CW_QUICK_STOP_MASK : 0) | ((eo) ? CIA402_CW_ENABLE_OPERATION_MASK : 0) |   \
	 ((fr) ? CIA402_CW_FAULT_RESET_MASK : 0))

#ifdef __cplusplus
}
#endif

#endif /* CIA402_DEFS_H */
