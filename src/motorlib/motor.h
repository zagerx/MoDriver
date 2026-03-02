#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "motor_driver.h"

struct motor;

/**
 * @brief 电机实例1
 */
extern struct motor *motor_1;

/**
 * @brief 批量绑定硬件接口（推荐）
 *
 * @param motor 电机实例
 * @param hw 硬件接口集合
 */
void motor_bind_hardware(struct motor *motor, const struct motor_hw_ops *hw);

/**
 * @brief 高频控制任务
 *
 * 应在定时器中断中周期性调用
 *
 * @param motor 电机实例
 */
void motor_highfreq_task(struct motor *motor);

void motor_init(struct motor *motor);

#endif /* MOTOR_H */
