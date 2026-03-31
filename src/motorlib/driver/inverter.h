

/**
 * @file inverter.h
 * @brief 逆变器驱动头文件
 * @details 实现三相逆变器的使能、禁用和电压设置功能
 */

#ifndef INVERTER_H
#define INVERTER_H

#include <stdint.h>

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

/**
 * @brief 使能逆变器
 * @param[in] inverter 逆变器实例
 * @return 无
 */
void inverter_enable(struct inverter *inverter);

/**
 * @brief 禁用逆变器
 * @param[in] inverter 逆变器实例
 * @return 无
 */
void inverter_disable(struct inverter *inverter);

/**
 * @brief 设置三相电压
 * @param[in] inverter 逆变器实例
 * @param[in] u U相电压
 * @param[in] v V相电压
 * @param[in] w W相电压
 * @return 无
 */
void inverter_set_voltage(struct inverter *inverter, float u, float v, float w);

#endif /* INVERTER_H */
