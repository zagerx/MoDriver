#ifndef CURRSMP_H
#define CURRSMP_H
#include <stdint.h>
#include "motor_driver.h"
struct currsmp_input {
	uint16_t i_a_raw;   /**< @brief a轴电流采样原始值 */
	uint16_t i_b_raw;   /**< @brief b轴电流采样原始值 */
	uint16_t i_c_raw;   /**< @brief c轴电流采样原始值 */
	uint16_t i_bus_raw; /**< @brief 母线电流采样原始值 */
	uint16_t v_bus_raw; /**< @brief 母线电压采样原始值 */
};
struct currsmp_output {
	float i_a;   /**< @brief a轴电流采样值 */
	float i_b;   /**< @brief b轴电流采样值 */
	float i_c;   /**< @brief c轴电流采样值 */
	float i_bus; /**< @brief 母线电流采样值 */
	float v_bus; /**< @brief 母线电压采样值 */
};
struct currsmp_data {
	float test; /**< @brief 测试数据 */
};
/**
 * @brief 电流采样数据结构体
 */
struct currsmp {
	struct currsmp_input input;   /**< @brief 电流采样原始数据 */
	struct currsmp_output output; /**< @brief 电流采样数据 */
	struct currsmp_param *param;  /**< @brief 电流采样参数 */
	struct currsmp_data data;     /**< @brief 电流采样内部数据 */
};

/**
 * @brief 绑定电流采样参数
 * @param[in] currsmp 电流采样实例
 * @param[in] param 电流采样参数
 * @return 无
 */
void currsmp_bind_param(struct currsmp *currsmp, struct currsmp_param *param);
/**
 * @brief 更新电流采样原始数据
 * @param[in] currsmp 电流采样实例
 * @param[in] adc_raw ADC原始数据数组
 * @return 无
 * @note adc_raw数组顺序: [i_a_raw, i_b_raw, i_c_raw, i_bus_raw, v_bus_raw]
 */
void currsmp_update_raw(struct currsmp *currsmp, uint16_t *adc_raw);
/**
 * @brief 更新电流采样计算值（转换为物理量）
 * @param[in] currsmp 电流采样实例
 * @return 无
 * @details 使用原始值和参数计算相电流、母线电流、母线电压
 */
void currsmp_update(struct currsmp *currsmp);
/**
 * @brief 仅更新母线电压和电流
 * @param[in] currsmp 电流采样实例
 * @return 无
 * @note 在校准状态下使用，仅计算母线相关物理量
 */
void currsmp_update_bus(struct currsmp *currsmp);

/**
 * @brief 获取电流采样输出数据
 * @param[in] currsmp 电流采样实例
 * @param[out] output 输出数据指针
 * @return 无
 */
static inline void currsmp_get_output(struct currsmp *currsmp, struct currsmp_output *output)
{
	if (!currsmp || !output) {
		return;
	}
	*output = currsmp->output;
}
/**
 * @brief 获取电流采样原始数据
 * @param[in] currsmp 电流采样实例
 * @param[out] input 原始数据指针
 * @return 无
 */
static inline void currsmp_get_raw(struct currsmp *currsmp, struct currsmp_input *input)
{
	if (!currsmp || !input) {
		return;
	}
	*input = currsmp->input;
}

/**
 * @brief 更新电流采样通道偏移量
 * @param[in] currsmp 电流采样实例
 * @param[in] adc_raw ADC原始数据数组（三相电流）
 * @return 无
 * @note 用于电流校准，将当前ADC值设为偏移量
 */
static inline void currsmp_update_offset(struct currsmp *currsmp, uint16_t *adc_raw)
{
	if (!currsmp || !currsmp->param) {
		return;
	}
	currsmp->param->a_chn_offset = adc_raw[0];
	currsmp->param->b_chn_offset = adc_raw[1];
	currsmp->param->c_chn_offset = adc_raw[2];
}

/**
 * @brief 更新相电流增益
 * @param[in] currsmp 电流采样实例
 * @param[in] gain_phase 相电流增益系数
 * @return 无
 */
static inline void currsmp_update_phase_gain(struct currsmp *currsmp, float gain_phase)
{
	if (!currsmp || !currsmp->param) {
		return;
	}
	currsmp->param->gain_phase = gain_phase;
}
/**
 * @brief 更新母线电流增益
 * @param[in] currsmp 电流采样实例
 * @param[in] gain_i_bus 母线电流增益系数
 * @return 无
 */
static inline void currsmp_update_i_bus_gain(struct currsmp *currsmp, float gain_i_bus)
{
	if (!currsmp || !currsmp->param) {
		return;
	}
	currsmp->param->gain_i_bus = gain_i_bus;
}
/**
 * @brief 更新母线电压增益
 * @param[in] currsmp 电流采样实例
 * @param[in] gain_v_bus 母线电压增益系数
 * @return 无
 */
static inline void currsmp_update_v_bus_gain(struct currsmp *currsmp, float gain_v_bus)
{
	if (!currsmp || !currsmp->param) {
		return;
	}
	currsmp->param->gain_v_bus = gain_v_bus;
}
#endif /* CURRSMP_H */
