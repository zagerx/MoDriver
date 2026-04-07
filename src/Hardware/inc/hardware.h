#ifndef HARDWARE_H
#define HARDWARE_H

#include "encoder.h"
#include "pwm.h"
#include "tim.h"
#include "adc.h"
void hardware_init(void);
void hardware_start_irq(void);

#endif
