#include "inverter.h"

void inverter_bind_inverter(struct inverter *inverter, const struct inverter_ops *ops)
{
	if (inverter) {
		inverter->ops = ops;
	}
}
