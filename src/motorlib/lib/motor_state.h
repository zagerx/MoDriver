
/**
 * @file motor_state.h
 * @brief 电机主状态机头文件
 * @details 定义电机主运行状态（INIT/CALIB/IDLE/RUNING）及状态切换接口
 */

#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include "statemachine.h"
struct motor;

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
void motor_calib_state(struct statemachine *sm);

/**
 * @brief 电机空闲状态
 * @param[in] sm 状态机实例
 * @return 无
 */
void motor_idle_state(struct statemachine *sm);

/**
 * @brief 电机运行状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 电机正常运行状态，执行闭环控制算法
 */
void motor_runing_state(struct statemachine *sm);

#endif /* MOTOR_STATE_H */
