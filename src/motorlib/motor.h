#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
struct motor;
extern  struct motor *motor_1;   
void motor_higfre_task(struct motor *motor);  
void motor_register_callback(struct motor *motor, uint16_t (*cb)(void), void (*disable)(void),
			     void (*enable)(void), void (*set)(float, float, float));
#endif
