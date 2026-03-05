#include "inverter.h"

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

void inverter_enable(struct inverter *inverter)
{
	if (inverter && inverter->ops && inverter->ops->enable) {
		inverter->ops->enable();
	}
}

void inverter_disable(struct inverter *inverter)
{
	if (inverter && inverter->ops && inverter->ops->disable) {
		inverter->ops->disable();
	}
}

void inverter_set_voltage(struct inverter *inverter, float u, float v, float w)
{
	if (inverter->ops->set_voltage) {
		inverter->ops->set_voltage(u, v, w);
	}
}