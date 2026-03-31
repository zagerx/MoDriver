

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "motor_driver.h"

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
 * @note 应在定时器中断中周期性调用（默认10kHz），执行反馈更新和状态机调度
 */
void motor_highfreq_task(struct motor *motor, uint16_t *adc_raw);
void motor_tran_runing(struct motor *motor);
void motor_tran_idle(struct motor *motor);
void motor_tran_pp_mode(struct motor *motor);
void motor_tran_pv_mode(struct motor *motor);
void motor_tran_none_mode(struct motor *motor);
void motor_set_target_pos(struct motor *motor, float target_pos, float target_vel);

#endif /* MOTOR_H */
