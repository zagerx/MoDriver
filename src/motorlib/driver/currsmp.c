#include "currsmp.h"

/**
 * @brief 绑定电流采样参数
 * @param[in] currsmp 电流采样实例
 * @param[in] param 电流采样参数
 * @return 无
 */
void currsmp_bind_param(struct currsmp *currsmp, struct currsmp_param *param)
{
	if (currsmp) {
		currsmp->param = param;
	}
}
/**
 * @brief 初始化电流采样实例
 * @param[in] currsmp 电流采样实例
 * @return 无
 * @note 清零所有原始数据和输出数据
 */
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
/**
 * @brief 更新电流采样原始数据
 * @param[in] currsmp 电流采样实例
 * @param[in] adc_raw ADC原始数据数组
 * @return 无
 * @details 将ADC原始数据赋值给电流采样实例的输入结构
 */
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
/**
 * @brief 更新电流采样计算值（转换为物理量）
 * @param[in] currsmp 电流采样实例
 * @return 无
 * @details 使用原始值和参数计算相电流、母线电流、母线电压
 */
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
/**
 * @brief 仅更新母线电压和电流
 * @param[in] currsmp 电流采样实例
 * @return 无
 * @note 在校准状态下使用，仅计算母线相关物理量
 */
void currsmp_update_bus(struct currsmp *currsmp)
{
	if (!currsmp) {
		return;
	}
	currsmp->output.v_bus = (currsmp->input.v_bus_raw) * currsmp->param->gain_v_bus;
	currsmp->output.i_bus = (currsmp->input.i_bus_raw) * currsmp->param->gain_i_bus;
}