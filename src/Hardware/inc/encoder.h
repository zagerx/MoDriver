#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

uint16_t encoder_getraw(void);
void encoder_init_pipeline(void);

#endif
