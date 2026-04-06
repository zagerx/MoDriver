#ifndef CIA402_H
#define CIA402_H
#include <stdint.h>
#include <stdbool.h>
#include "statemachine.h"

struct motor;
struct cia402_instance {
	/* ========== OD 变量指针 (绑定到 OD_RAM) ========== */
	uint16_t *controlword;      /*!< 0x6040 - 控制字 */
	uint16_t *statusword;       /*!< 0x6041 - 状态字 */
	int8_t *modes_of_operation; /*!< 0x6060 - 操作模式 */
	int8_t *mode_display;       /*!< 0x6061 - 模式显示 */
	uint16_t *error_code;       /*!< 0x603F - 错误码 */

	/* 目标值 - RPDO 映射 */
	int32_t *target_position;  /*!< 0x607A - 目标位置 */
	int32_t *target_velocity;  /*!< 0x60FF - 目标速度 */
	int16_t *target_torque;    /*!< 0x6071 - 目标转矩 */
	int32_t *profile_velocity; /*!< 0x6081 - 轮廓速度 */
	/* 实际值 - TPDO 映射 */
	int32_t *actual_position; /*!< 0x6064 - 实际位置 */
	int32_t *actual_velocity; /*!< 0x606C - 实际速度 */
	int16_t *actual_torque;   /*!< 0x6077 - 实际转矩 */

	/* 配置参数 */
	uint32_t supported_modes;  /*!< 0x6502 - 支持的模式位图 */
	uint16_t quick_stop_decel; /*!< 0x6085 - 快速停止减速度 */

	/* ========== 状态机 ========== */
	struct statemachine pds_sm;
	/* ========== 关联的 motor 实例 ========== */
	struct motor *motor;

	/* ========== 内部标志 ========== */
	uint16_t cache_controlword; /*!< 内部缓存的控制字值，用于检测变化 */
	uint8_t cache_mode;         /*!< 内部缓存的操作模式值 */
	bool is_initialized;
	bool halt_active;    /*!< 暂停标志 */
	uint16_t fault_code; /*!< 内部故障码缓存 */
};

void cia402_params_bind(struct cia402_instance *instance, struct motor *motor,
			uint16_t *controlword, uint16_t *statusword, int8_t *modes_of_operation,
			int8_t *mode_display, int32_t *target_velocity, int32_t *actual_velocity,
			uint16_t *error_code, int32_t *target_position, int16_t *target_torque,
			int32_t *actual_position, int16_t *actual_torque,
			int32_t *profile_velocity);
void cia402_init(struct cia402_instance *instance);
void cia402_update(struct cia402_instance *instance, float dt);
#endif /* CIA402_H */
