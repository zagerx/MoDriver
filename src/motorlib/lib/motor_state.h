

#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include "statemachine.h"
struct motor;
enum motor_status {
	MOTOR_STATUS_INIT = 0,
	MOTOR_STATUS_CALIB,
	MOTOR_STATUS_IDLE,
	MOTOR_STATUS_RUNING,
	MOTOR_STATUS_MAX,
};
/**
 * @brief 电机初始化状态
 * @param[in] sm 状态机实例
 * @return 无
 */
void motor_init_state(struct statemachine *sm);

/**
 * @brief 电机校准状态
 * @param[in] sm 状态机实例
 * @return 无
 */
void motor_carib_state(struct statemachine *sm);

/**
 * @brief 电机空闲状态
 * @param[in] sm 状态机实例
 * @return 无
 */
void motor_idle_state(struct statemachine *sm);

/**
 * @brief 电机运行状态
 * @param[in] sm 状态机实例
 * @return 无
 */
/**
 * @brief 电机运行状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 电机正常运行状态，执行闭环控制算法
 */
void motor_runing_state(struct statemachine *sm);

/**
 * @brief 切换电机主状态
 * @param[in] motor 电机实例指针
 * @param[in] new_state 新的电机状态
 * @details 内部函数，用于状态机切换主运行状态
 */
void _tran_state(struct motor *motor, enum motor_status new_state);

#endif /* MOTOR_STATE_H */
