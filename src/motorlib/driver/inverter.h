#ifndef INVERTER_H
#define INVERTER_H

#include <stdint.h>
#include "motor_driver.h"
/**
 * @brief 逆变器控制结构体
 */
struct inverter {
	const struct inverter_ops *ops;
};

/**
 * @param inverter 逆变器实例
 * @param disable 禁用函数
 * @param enable 使能函数
 * @param set 设置电压函数
 */
void inverter_bind_inverter(struct inverter *inverter, const struct inverter_ops *ops);

#endif /* INVERTER_H */
