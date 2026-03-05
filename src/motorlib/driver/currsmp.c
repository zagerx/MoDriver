#include "currsmp.h"

void currsmp_bind_param(struct currsmp *currsmp, struct currsmp_param *param)
{
	if (currsmp) {
		currsmp->param = param;
	}
}
void currsmp_init(struct currsmp *currsmp)
{
	if (!currsmp || !currsmp->param) {
		return;
	}
	currsmp->input.i_a_raw = 0;
	currsmp->input.i_b_raw = 0;
	currsmp->input.i_c_raw = 0;
	currsmp->input.i_bus_raw = 0;
	currsmp->input.v_bus_raw = 0;

	currsmp->output.i_a = 0.0f;
	currsmp->output.i_b = 0.0f;
	currsmp->output.i_c = 0.0f;
	currsmp->output.i_bus = 0.0f;
	currsmp->output.v_bus = 0.0f;
}
void currsmp_update_raw(struct currsmp *currsmp, uint16_t *adc_raw)
{
	if (!currsmp) {
		return;
	}
	currsmp->input.i_a_raw = adc_raw[0];
	currsmp->input.i_b_raw = adc_raw[1];
	currsmp->input.i_c_raw = adc_raw[2];
	currsmp->input.i_bus_raw = adc_raw[3];
	currsmp->input.v_bus_raw = adc_raw[4];
}
void currsmp_update(struct currsmp *currsmp)
{
	if (!currsmp) {
		return;
	}
	/* 转换为物理量 */
	currsmp->output.i_a = (currsmp->input.i_a_raw - currsmp->param->a_chn_offset) *
			      currsmp->param->gain_phase;
	currsmp->output.i_b = (currsmp->input.i_b_raw - currsmp->param->b_chn_offset) *
			      currsmp->param->gain_phase;
	currsmp->output.i_c = (currsmp->input.i_c_raw - currsmp->param->c_chn_offset) *
			      currsmp->param->gain_phase;
	currsmp->output.i_bus = (currsmp->input.i_bus_raw) * currsmp->param->gain_i_bus;
	currsmp->output.v_bus = (currsmp->input.v_bus_raw) * currsmp->param->gain_v_bus;
}
