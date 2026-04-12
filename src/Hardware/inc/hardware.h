#ifndef HARDWARE_H
#define HARDWARE_H

#include "encoder.h"
#include "pwm.h"
#include "tim.h"
#include "adc.h"

void hardware_flash_write_params(uint8_t *data, uint16_t *datalen);
void hardware_flash_read_params(uint8_t *data, uint16_t *datalen);

void hardware_init(void);
void hardware_start_irq(void);

#endif
