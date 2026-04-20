/**
 * @file close_loop.h
 * @brief 电机闭环控制头文件
 * @details 实现电机三环控制（位置环、速度环、电流环）
 */

#ifndef CLOSE_LOOP_H
#define CLOSE_LOOP_H

struct motor;
float motor_position_loop(struct motor *motor, float tar_pos, float dt);

/**
 * @brief 电机位置环控制
 * @param[in] motor 电机实例指针
 * @param[in] dt 时间步长（单位：秒）
 * @details 执行位置闭环控制，读取轨迹规划位置并与实际位置做PID运算，输出速度指令
 * @note 位置环输出 = 位置PID计算结果 + 轨迹规划速度前馈
 */
// void motor_position_loop(struct motor *motor, float dt);

/**
 * @brief 复位位置环控制器
 * @param[in] motor 电机实例指针
 * @details 复位位置环、速度环、电流环的PID控制器状态，并清零参考值
 * @note 通常在模式切换或故障恢复时调用
 */
void motor_position_loop_reset(struct motor *motor);

/**
 * @brief 电机速度环控制
 * @param[in] motor 电机实例指针
 * @param[in] target_vel 目标速度（单位：rad/s）
 * @details 执行速度闭环控制，根据目标速度和实际速度计算q轴电流参考值
 * @note 速度环输出为q轴电流参考值，d轴电流参考值设为0（Id=0控制）
 */
void motor_velocity_loop(struct motor *motor, float target_vel);

/**
 * @brief 复位速度环控制器
 * @param[in] motor 电机实例指针
 * @details 复位速度环和电流环的PID控制器状态，并清零电流参考值
 * @note 通常在速度环退出或故障恢复时调用
 */
void motor_velocity_loop_reset(struct motor *motor);

/**
 * @brief 电机电流环控制
 * @param[in] motor 电机实例指针
 * @details 执行电流闭环控制，计算电压指令（d/q轴电压），经SVPWM输出到逆变器
 * @note 包含电压限幅和前馈解耦，是三环控制的最内环
 */
void motor_currment_loop(struct motor *motor);

/**
 * @brief 复位电流环控制器
 * @param[in] motor 电机实例指针
 * @details 复位d/q轴电流环的PID控制器状态，并清零电流参考值
 * @note 通常在电流环退出或逆变器关闭时调用
 */
void motor_current_loop_reset(struct motor *motor);

#endif /* CLOSE_LOOP_H */
