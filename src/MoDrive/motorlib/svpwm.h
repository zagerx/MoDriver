#ifndef SVPWM_H
#define SVPWM_H

void svm_set(float va, float vb, float *dabc);
void svpwm_seven_segment(float v_alpha, float v_beta, float *duty_a, float *duty_b, float *duty_c);
#endif
