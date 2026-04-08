
/**
 * @file motor.h
 * @brief 电机控制模块公共头文件
 * @details 提供电机初始化、硬件绑定、高频任务调度及状态查询API
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "motor_interface.h"

struct motor;

/** @brief 电机实例1 */
extern struct motor *motor_1;

/**
 * @brief 绑定硬件接口
 * @param[in] motor 电机实例
 * @param[in] hw 硬件接口集合（编码器、逆变器）
 * @return 无
 * @note 将硬件层的操作接口绑定到电机实例
 */
void motor_bind_hardware(struct motor *motor, const struct motor_hw_ops *hw);

/**
 * @brief 绑定扩展参数
 * @param[in] motor 电机实例
 * @param[in] param_ext 扩展参数（包含反馈参数及CRC）
 * @return 无
 * @note 绑定电机运行时所需的参数配置
 */
void motor_bind_param_ext(struct motor *motor, struct motor_param_ext *param_ext);

/**
 * @brief 初始化电机
 * @param[in] motor 电机实例
 * @return 无
 * @note 检查参数并初始化状态机，根据检查结果进入相应初始状态
 */
void motor_init(struct motor *motor);

/**
 * @brief 高频控制任务
 * @param[in] motor 电机实例
 * @param[in] adc_raw ADC原始数据数组
 * @return 无
 * @note 应在定时器中断中周期性调用，执行反馈更新和状态机调度
 */
void motor_highfreq_task(struct motor *motor, uint16_t *adc_raw);

/*
 * 以下位标志相关API在 motor_interface_bits.h 中定义：
 * - motor_get_flag / motor_set_flag / motor_clear_flag / motor_clear_all_flags
 * - motor_is_flag_set
 * - motor_get_errorcode / motor_set_error / motor_clear_error / motor_clear_all_errors
 * - motor_is_error_set
 * - motor_set_command / motor_clear_command / motor_clear_all_commands
 * - motor_is_command_set
 */

/**
 * @brief 获取电机当前操作模式
 * @param[in] motor 电机实例
 * @return enum motor_mode 当前操作模式
 * @note 通过模式状态机 current_state 反查枚举值
 */
enum motor_mode motor_get_mode(const struct motor *motor);

/**
 * @brief 切换电机操作模式
 * @param[in] motor 电机实例
 * @param[in] new_mode 新的操作模式
 * @return 无
 * @details 内部函数，用于状态机切换操作模式
 */
void motor_tran_mode(struct motor *motor, enum motor_mode new_mode);

/**
 * @brief 获取电机当前主状态
 * @param[in] motor 电机实例
 * @return enum motor_status 当前主状态
 * @note 通过状态机 current_state 反查枚举值
 */
enum motor_status motor_get_status(const struct motor *motor);

/**
 * @brief 切换电机主状态
 * @param[in] motor 电机实例
 * @param[in] new_state 新的电机主状态
 * @return 无
 * @details 用于状态机切换电机主运行状态（INIT/CALIB/IDLE/RUNING）
 */
void motor_tran_state(struct motor *motor, enum motor_status new_state);

/**
 * @brief 设置电机目标位置与速度
 * @param[in] motor 电机实例
 * @param[in] target_pos 目标位置
 * @param[in] target_vel 目标速度
 * @return 无
 * @details 更新轨迹规划器的目标位置与速度，用于PP模式
 */
void motor_set_target_pos(struct motor *motor, float target_pos, float target_vel);

void motor_stop(struct motor *motor);
void motor_enable(struct motor *motor);
void motor_disable(struct motor *motor);
void motor_get_all_data(const struct motor *motor, struct motor_all_state *state);

#include "stdbool.h"
extern bool motor_protection_has_fault(struct motor *motor);

#endif /* MOTOR_H */
