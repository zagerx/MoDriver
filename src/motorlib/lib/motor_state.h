#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H
#include "statemachine.h"
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
void motor_runing_state(struct statemachine *sm);

#endif /* MOTOR_STATE_H */