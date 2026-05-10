#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include <stdint.h>

struct motor;

/**
 * @brief 驱动器专有指令实例
 */
struct drive_command_instance {
	/* ========== OD 变量指针 (绑定到 OD_RAM) ========== */
	uint8_t *cmd_id; /*!< 0x3000:01 - 指令码 */
	float *arg1;     /*!< 0x3000:02 - 参数 */

	/* ========== 关联的 motor 实例 ========== */
	struct motor *motor;

	/* ========== 内部状态 ========== */
	uint8_t cache_cmd; /*!< 缓存的上一次指令码，用于检测变化 */
};

/* 指令码定义：一码一令，扁平化 */
#define DCMD_MOTOR_TRAN_RUNING      0x01U /*!< 电机进入运行状态指令 */
#define DCMD_MOTOR_TARN_IDLE        0x02U /*!< 电机进入空闲状态指令 */
#define DCMD_MOTOR_TARN_CALIB       0x03U /*!< 电机进入校准状态指令 */
#define DCMD_MOTOR_TARN_MODE_DEBUG  0x04U /*!< 电机进入调试模式指令 */
#define DCMD_MOTOR_RAN_MODE_V       0x06U /*!< 电机进入位置模式指令 */
#define DCMD_MOTOR_RAN_MODE_P       0x07U /*!< 电机进入速度模式指令 */
#define DCMD_MOTOR_TRAN_ANTICOGGING 0x08U /*!< 电机进入齿槽补偿校准模式指令 */
#define DCMD_SET_VELEOIVER_TARVAL   0x10U /*!< 设置目标速度指令 */
#define DCMD_SET_POSITON_TARVAL     0x11U /*!< 设置目标位置指令 */

/**
 * @brief 绑定驱动器指令参数
 * @param[in,out] inst 指令实例
 * @param[in] motor 电机实例
 * @param[in] cmd_id 指令码指针 (0x3000:01)
 * @param[in] arg1 参数指针 (0x3000:02)
 * @return 无
 */
void drive_command_params_bind(struct drive_command_instance *inst, struct motor *motor,
			       uint8_t *cmd_id, float *arg1);

/**
 * @brief 初始化驱动器指令实例
 * @param[in,out] inst 指令实例
 * @return 无
 */
void drive_command_init(struct drive_command_instance *inst);

/**
 * @brief 更新处理驱动器专有指令
 * @param[in,out] inst 指令实例
 * @return 无
 * @note 应在主循环中周期性调用，检测指令变化并执行
 */
void drive_command_update(struct drive_command_instance *inst);

#endif /* DRIVE_COMMAND_H */
