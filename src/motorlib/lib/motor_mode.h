#ifndef MOTOR_MODE_H
#define MOTOR_MODE_H
#include "statemachine.h"
struct statemachine;
struct motor;
/**
 * @brief CiA 402 操作模式定义
 * @note 对应对象字典 0x6060 (Modes of Operation) 和 0x6061 (Modes of Operation Display)
 */
enum motor_mode {
	/* 标准未定义/无模式 */
	MODE_NONE = 0x00, /*!< 无模式分配 (Not assigned) */

	/* 轮廓模式 (Profile Modes) - 非同步，内部规划轨迹 */
	MODE_PP = 0x01, /*!< 轮廓位置模式 (Profile Position Mode) */
	MODE_PV = 0x03, /*!< 轮廓速度模式 (Profile Velocity Mode) */
	MODE_PT = 0x04, /*!< 轮廓转矩模式 (Profile Torque Mode) - 较少使用 */
	MODE_HM = 0x06, /*!< 原点回归模式 (Homing Mode) */

	/* 循环同步模式 (Cyclic Synchronous Modes) - 同步，主站规划轨迹 */
	MODE_IP = 0x07,  /*!< 插补位置模式 (Interpolated Position Mode) */
	MODE_CSP = 0x08, /*!< 循环同步位置模式 (Cyclic Synchronous Position Mode) */
	MODE_CSV = 0x09, /*!< 循环同步速度模式 (Cyclic Synchronous Velocity Mode) */
	MODE_CST = 0x0A, /*!< 循环同步转矩模式 (Cyclic Synchronous Torque Mode) */

	/* 扩展模式 */
	MODE_CSTCA = 0x0B, /*!< 循环同步转矩带通讯角模式 (CST with Commutation Angle) */

	/* 制造商特定 (0x7F-0xFF 保留) */
	MODE_MANUFACTURER = 0xFF, /*!< 制造商特定模式 */
};
void motor_mode_none(struct statemachine *sm);
void motor_mode_HOMING(struct statemachine *sm);
void motor_mode_PP(struct statemachine *sm);
void motor_mode_PV(struct statemachine *sm);
void _tran_mode(struct motor *motor, enum motor_mode new_mode);

#endif
