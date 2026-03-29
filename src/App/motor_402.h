#ifndef MOTOR_402_H
#define MOTOR_402_H
#include <stdint.h>
#include <stdbool.h>
#include "statemachine.h"

struct cia402_instance {
	/* ========== OD 变量指针 (绑定到 OD_RAM) ========== */
	uint16_t *controlword;      /*!< 0x6040 - 控制字 */
	uint16_t *statusword;       /*!< 0x6041 - 状态字 */
	int8_t *modes_of_operation; /*!< 0x6060 - 操作模式 */
	int8_t *mode_display;       /*!< 0x6061 - 模式显示 */
	uint16_t *error_code;       /*!< 0x603F - 错误码 */

	/* 目标值 - RPDO 映射 */
	int32_t *target_position; /*!< 0x607A - 目标位置 */
	int32_t *target_velocity; /*!< 0x60FF - 目标速度 */
	int16_t *target_torque;   /*!< 0x6071 - 目标转矩 */

	/* 实际值 - TPDO 映射 */
	int32_t *actual_position; /*!< 0x6064 - 实际位置 */
	int32_t *actual_velocity; /*!< 0x606C - 实际速度 */
	int16_t *actual_torque;   /*!< 0x6077 - 实际转矩 */

	/* 配置参数 */
	uint32_t supported_modes;  /*!< 0x6502 - 支持的模式位图 */
	uint16_t quick_stop_decel; /*!< 0x6085 - 快速停止减速度 */

	/* ========== 状态机 ========== */
	struct statemachine pds_sm; /*!< PDS 状态机实例 */
	/* ========== 关联的 motor 实例 ========== */
	struct motor *motor; /*!< 关联的电机实例 */

	/* ========== 内部标志 ========== */
	bool is_initialized;
	bool halt_active;    /*!< 暂停标志 */
	uint16_t fault_code; /*!< 内部故障码缓存 */
};

#endif /* MOTOR_402_H */
