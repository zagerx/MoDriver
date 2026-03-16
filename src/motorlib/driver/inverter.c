#include "inverter.h"
#include "motor_driver.h"
#include "motorlib_control_param.h"
/**
 * @brief 逆变器控制结构体
 */
struct inverter {
	const struct inverter_ops *ops; /**< @brief 逆变器操作接口 */
};

#if !defined(MOTOR_COUNT) || (MOTOR_COUNT == 0)
#error "MOTOR_COUNT not defined or invalid"
#elif MOTOR_COUNT == 1
struct inverter inverter_1;
#elif MOTOR_COUNT == 2
struct inverter inverter_1;
struct inverter inverter_2;
#else
#error "MOTOR_COUNT must be 1 or 2"
#endif
/**
 * @brief 绑定逆变器操作接口
 * @param[in] inverter 逆变器实例
 * @param[in] ops 逆变器操作接口
 * @return 无
 */
void inverter_bind_inverter(struct inverter *inverter, const struct inverter_ops *ops)
{
	if (inverter) {
		inverter->ops = ops;
	}
}

/**
 * @brief 使能逆变器
 * @param[in] inverter 逆变器实例
 * @return 无
 */
void inverter_enable(struct inverter *inverter)
{
	if (inverter && inverter->ops && inverter->ops->enable) {
		inverter->ops->enable();
	}
}

/**
 * @brief 禁用逆变器
 * @param[in] inverter 逆变器实例
 * @return 无
 */
void inverter_disable(struct inverter *inverter)
{
	if (inverter && inverter->ops && inverter->ops->disable) {
		inverter->ops->disable();
	}
}

/**
 * @brief 设置三相电压
 * @param[in] inverter 逆变器实例
 * @param[in] u U相电压
 * @param[in] v V相电压
 * @param[in] w W相电压
 * @return 无
 */
void inverter_set_voltage(struct inverter *inverter, float u, float v, float w)
{
	if (inverter->ops->set_voltage) {
		inverter->ops->set_voltage(u, v, w);
	}
}