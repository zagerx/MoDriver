/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "motor_driver.h"

struct motor;

/* 电机实例1 */
extern struct motor *motor_1;

/**
 * motor_bind_hardware - 绑定硬件接口
 * @motor: 电机实例
 * @hw: 硬件接口集合（编码器、逆变器）
 *
 * 将硬件层的操作接口绑定到电机实例
 */
void motor_bind_hardware(struct motor *motor, const struct motor_hw_ops *hw);

/**
 * motor_bind_param_ext - 绑定扩展参数
 * @motor: 电机实例
 * @param_ext: 扩展参数（包含反馈参数及CRC）
 *
 * 绑定电机运行时所需的参数配置
 */
void motor_bind_param_ext(struct motor *motor, struct motor_param_ext *param_ext);

/**
 * motor_param_check - 检查电机参数合法性
 * @motor: 电机实例
 *
 * Return:
 * * 0      - 参数合法
 * * -1     - 电机实例为空
 * * -2     - 扩展参数为空
 * * -3     - 反馈参数为空
 * * -10~-14 - 具体参数值非法
 */
int16_t motor_param_check(struct motor *motor);

/**
 * motor_init - 初始化电机
 * @motor: 电机实例
 *
 * 检查参数并初始化状态机，根据检查结果进入相应初始状态
 */
void motor_init(struct motor *motor);

/**
 * motor_highfreq_task - 高频控制任务
 * @motor: 电机实例
 *
 * 应在定时器中断中周期性调用（默认10kHz）
 * 执行反馈更新和状态机调度
 */
void motor_highfreq_task(struct motor *motor);

#endif /* MOTOR_H */
