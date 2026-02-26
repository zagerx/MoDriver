#ifndef INVERTER_H
#define INVERTER_H
#include <stdint.h>
struct inverter {
	void (*phase_volteage_enable)(void);
	void (*phase_volteage_disable)(void);
	void (*phase_volteage_set)(float, float, float);
};
void inverter_register_callback(struct inverter *inverter, void (*disable)(void),
				void (*enable)(void), void (*set)(float, float, float));
#endif
