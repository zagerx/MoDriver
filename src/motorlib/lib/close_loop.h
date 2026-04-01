/**
 * @file close_loop.h
 * @brief 电机闭环控制头文件
 * @details 实现电机三环控制（位置环、速度环、电流环）
 */

#ifndef CLOSE_LOOP_H
#define CLOSE_LOOP_H

struct motor;

/**
 * @brief 电机位置环控制
 * @param[in] motor 电机实例指针
 * @param[in] dt 时间步长
 * @details 执行位置闭环控制，计算速度指令
 */
void motor_position_loop(struct motor *motor, float dt);
void motor_position_loop_reset(struct motor *motor);

/**
 * @brief 电机速度环控制
 * @param[in] motor 电机实例指针
 * @details 执行速度闭环控制，计算电流指令（q轴电流）
 */
void motor_velocity_loop(struct motor *motor, float target_vel);
void motor_velocity_loop_reset(struct motor *motor);

/**
 * @brief 电机电流环控制
 * @param[in] motor 电机实例指针
 * @details 执行电流闭环控制，计算电压指令（d/q轴电压）
 */
void motor_currment_loop(struct motor *motor);
void motor_current_loop_reset(struct motor *motor);

#endif /* CLOSE_LOOP_H */
