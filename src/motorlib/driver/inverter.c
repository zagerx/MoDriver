#include "inverter.h"
void inverter_register_callback(struct inverter *inverter, void (*disable)(void),
				void (*enable)(void), void (*set)(float, float, float))
{
	inverter->phase_volteage_enable = enable;
	inverter->phase_volteage_disable = disable;
	inverter->phase_volteage_set = set;
}