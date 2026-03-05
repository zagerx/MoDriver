#ifndef INVERTER_H
#define INVERTER_H

#include <stdint.h>
#include "motor_driver.h"

/**
 * @brief 逆变器控制结构体
 */
struct inverter {
	const struct inverter_ops *ops; /**< @brief 逆变器操作接口 */
};

/**
 * @brief 绑定逆变器操作接口
 * @param[in] inverter 逆变器实例
 * @param[in] ops 逆变器操作接口
 * @return 无
 */
void inverter_bind_inverter(struct inverter *inverter, const struct inverter_ops *ops);
void inverter_enable(struct inverter *inverter);
void inverter_disable(struct inverter *inverter);
void inverter_set_voltage(struct inverter *inverter, float u, float v, float w);
#endif /* INVERTER_H */
