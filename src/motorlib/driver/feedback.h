
/**
 * @file feedback.h
 * @brief 编码器反馈模块头文件
 * @details 提供编码器读取、角度解算、速度计算及里程统计功能
 */

#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "motor_interface_driver.h"
#include "motor_interface_params.h"
#include "motorlib_constants.h"

/* 前向声明，避免循环包含 */
struct motor;

/* PLL默认带宽（Hz） */
#ifndef FEEDBACK_PLL_BANDWIDTH
#define FEEDBACK_PLL_BANDWIDTH 1000.0f
#endif

/**
 * @brief 反馈原始数据与中间计算数据结构体
 */
struct feedback_data {
	volatile uint16_t raw;         /**< @brief 原始编码器读数 */
	volatile uint16_t prev_raw;    /**< @brief 上一次原始读数（用于溢出检测） */
	volatile int32_t total_counts; /**< @brief 累积计数（shadow_count） */
	float odometer_offset_mangle;  /**< @brief 里程计零点偏移角度，单位：rad */

	/* PLL状态变量 */
	float pos_estimate_counts;  /**< @brief 位置估计（编码器计数） */
	float pos_cpr_counts;       /**< @brief CPR内位置估计（0~CPR-1） */
	float vel_estimate_counts;  /**< @brief 速度估计（计数/秒） */
	float pll_kp;               /**< @brief PLL比例增益 */
	float pll_ki;               /**< @brief PLL积分增益 */
	float delta_pos_cpr_counts; /**< @brief 相位检测器输出（调试用） */

	/* 电角度插值 */
	float phase_interp; /**< @brief [0,1) 编码器计数内插值 */
};

/**
 * @brief 反馈输出数据结构体
 */
struct feedback_output {
	float eangle_rad;     /**< @brief 电角度，单位：rad */
	float mangle_rad;     /**< @brief 机械角度，单位：rad */
	float velocity_rad_s; /**< @brief 机械角速度，单位：rad/s */
};

/**
 * @brief 编码器反馈结构体
 */
struct feedback {
	const struct encoder_ops *ops; /**< @brief 编码器操作接口 */
	struct feedback_param *param;  /**< @brief 反馈参数指针 */
	struct feedback_output output; /**< @brief 输出数据 */
	struct feedback_data data;     /**< @brief 原始数据与中间计算数据 */
};

/**
 * @brief 绑定编码器操作接口
 * @param[in] motor 电机实例
 * @param[in] ops 编码器操作接口
 * @return 无
 */
void feedback_bind_encoder(struct motor *motor, const struct encoder_ops *ops);

/**
 * @brief 绑定反馈参数
 * @param[in] motor 电机实例
 * @param[in] param 反馈参数
 * @return 无
 */
void feedback_bind_encoder_param(struct motor *motor, struct feedback_param *param);

/**
 * @brief 初始化反馈模块
 * @param[in] motor 电机实例
 * @return 无
 */
void feedback_init(struct motor *motor);

/**
 * @brief 重置编码器内部状态
 * @param[in] motor 电机实例
 * @return 无
 * @note 校准后调用，将当前编码器位置作为新零点，丢弃校准期间的位移
 */
void feedback_reset_encoder(struct motor *motor);

/**
 * @brief 更新反馈数据
 * @param[in] motor 电机实例
 * @param[in] dt 采样周期，单位：s
 * @return 无
 * @details 执行编码器读取、角度计算、速度计算
 */
void feedback_update(struct motor *motor, float dt);

/**
 * @brief 更新反馈原始数据
 * @param[in] motor 电机实例
 * @return 无
 */
void feedback_update_raw(struct motor *motor);

/**
 * @brief 获取编码器原始值
 * @param[in] fb 反馈实例
 * @return uint16_t 原始编码器读数
 */
static inline uint16_t feedback_get_raw(struct feedback *fb)
{
	return fb->ops ? fb->ops->read() : 0;
}

/**
 * @brief 获取电角度
 * @param[in] fb 反馈实例
 * @return float 电角度，单位：rad
 */
static inline float feedback_get_elec_angle(struct feedback *fb)
{
	return fb->output.eangle_rad;
}

/**
 * @brief 获取机械角速度
 * @param[in] fb 反馈实例
 * @return float 机械角速度，单位：rad/s
 */
static inline float feedback_get_velocity(struct feedback *fb)
{
	return fb->output.velocity_rad_s;
}

/**
 * @brief 更新旋转方向参数
 * @param[in] feedback 反馈实例
 * @param[in] direction 旋转方向（1 或 -1）
 * @return 无
 */
static inline void _feedback_update_param_direction(struct feedback *feedback, float direction)
{
	if (!feedback) {
		return;
	}

	feedback->param->direction = direction;
}

/**
 * @brief 更新编码器分辨率参数
 * @param[in] feedback 反馈实例
 * @param[in] encoder_resolution 编码器分辨率
 * @return 无
 */
static inline void _feedback_update_param_encoder_resolution(struct feedback *feedback,
							     uint16_t encoder_resolution)
{
	if (!feedback) {
		return;
	}

	feedback->param->encoder_resolution = encoder_resolution;
}

/**
 * @brief 更新编码器零位偏移参数
 * @param[in] feedback 反馈实例
 * @param[in] encoder_offset 编码器零位偏移整数部分
 * @param[in] encoder_offset_frac 编码器零位偏移小数部分
 * @return 无
 */
static inline void _feedback_update_param_encoder_offset(struct feedback *feedback,
							 uint16_t encoder_offset,
							 float encoder_offset_frac)
{
	if (!feedback) {
		return;
	}

	feedback->param->encoder_offset = encoder_offset;
	feedback->param->encoder_offset_frac = encoder_offset_frac;
}

/**
 * @brief 重置里程计
 * @param[in] feedback 反馈实例
 * @return 无
 * @note 记录当前角度作为偏移，实现相对零点。不会破坏角度连续性，可在运行时调用。
 */
static inline void feedback_reset_odometer(struct feedback *feedback)
{
	if (!feedback) {
		return;
	}

	/* 使用PLL位置估计计算当前机械角度作为里程零点 */
	const float cpr = (float)feedback->param->encoder_resolution;
	feedback->data.odometer_offset_mangle =
		(feedback->data.pos_estimate_counts - feedback->param->encoder_offset -
		 feedback->param->encoder_offset_frac) *
		(MOTORLIB_TWOPI / cpr) * feedback->param->direction;
}
#endif /* FEEDBACK_H */
