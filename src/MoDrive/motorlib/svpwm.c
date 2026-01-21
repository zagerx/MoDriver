/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <math.h>
#define FLOAT_EPS 1e-6f
#undef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
static inline float clamp_0_1(float x)
{
	if (x < 0.0f) {
		return 0.0f;
	}
	if (x > 1.0f) {
		return 1.0f;
	}
	return x;
}
/*
 * Seven-Segment SVPWM (Projection Method)
 */
void svpwm_seven_segment(float v_alpha, float v_beta, float *duty_a, float *duty_b, float *duty_c)
{
	/* ----------- 奇点处理 ----------- */
	if (fabsf(v_alpha) < FLOAT_EPS && fabsf(v_beta) < FLOAT_EPS) {
		*duty_a = 0.5f;
		*duty_b = 0.5f;
		*duty_c = 0.5f;
		return;
	}

	/* ----------- αβ → abc 投影（α 轴对齐 A 相） ----------- */
	float va = v_alpha;
	float vb = -0.5f * v_alpha + 0.8660254037844386f * v_beta; // +sqrt(3)/2
	float vc = -0.5f * v_alpha - 0.8660254037844386f * v_beta;

	/* ----------- 排序：max / mid / min ----------- */
	float vmax = fmaxf(va, fmaxf(vb, vc));
	float vmin = fminf(va, fminf(vb, vc));

	/* ----------- 关键增益校准（封版） ----------- */
	const float GAIN = 2.0f / 3.0f;

	float T1_plus_T2 = (vmax - vmin) * GAIN;
	T1_plus_T2 = clamp_0_1(T1_plus_T2);

	float T0 = 1.0f - T1_plus_T2;
	T0 = clamp_0_1(T0);

	float offset = 0.5f * T0;

	/* ----------- 七段式等效占空比 ----------- */
	float da = (va - vmin) * GAIN + offset;
	float db = (vb - vmin) * GAIN + offset;
	float dc = (vc - vmin) * GAIN + offset;

	*duty_a = clamp_0_1(da);
	*duty_b = clamp_0_1(db);
	*duty_c = clamp_0_1(dc);
}
