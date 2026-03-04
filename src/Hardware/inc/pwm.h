#ifndef PWM_H
#define PWM_H

void tim1_pwm_enable(void);
void tim1_pwm_disable(void);
void tim1_pwm_set_duty(float duty_a, float duty_b, float duty_c);

#endif
