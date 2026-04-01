
/**
 * @file feedback.h
 * @brief 编码器反馈模块头文件
 * @details 提供编码器读取、角度解算、速度计算及里程统计功能
 */

#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "motor_driver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** @brief 反馈错误码枚举 */
enum feedback_error_code {
	FEEDBACK_ERROR_NONE = 0,       /**< @brief 无错误 */
	FEEDBACK_ERROR_HW_FAILURE = 1, /**< @brief 硬件故障 */
	FEEDBACK_ERROR_PARAM = 2,      /**< @brief 参数错误 */
};

/** @brief 反馈状态枚举 */
enum feedback_state {
	FEEDBACK_STATE_OK = 0,             /**< @brief 正常状态 */
	FEEDBACK_STATE_NOT_CALIBRATED = 1, /**< @brief 未校准状态 */
};

/**
 * @brief 反馈原始数据与中间计算数据结构体
 */
struct feedback_data {
	volatile uint16_t raw;                 /**< @brief 原始编码器读数 */
	volatile uint16_t prev_raw;            /**< @brief 上一次原始读数（用于差分） */
	volatile int32_t total_counts;         /**< @brief 累积计数（考虑溢出） */
	volatile float accumulated_mangle_rad; /**< @brief 累积机械角度，单位：rad */
	volatile float prev_mangle_rad;  /**< @brief 上一次机械角度（用于速度差分），单位：rad */
	volatile float mech_omega_rad_s; /**< @brief 机械角速度，单位：rad/s */
};

/**
 * @brief 反馈输出数据结构体
 */
struct feedback_output {
	float eangle_rad;     /**< @brief 电角度，单位：rad */
	float velocity_rad_s; /**< @brief 机械角速度，单位：rad/s */
	float odometer;       /**< @brief 里程（累积线位移），单位：m */
};

/**
 * @brief 编码器反馈结构体
 */
struct feedback {
	const struct encoder_ops *ops; /**< @brief 编码器操作接口 */
	struct feedback_param *param;  /**< @brief 反馈参数指针 */
	struct feedback_output output; /**< @brief 输出数据 */
	struct feedback_data data;     /**< @brief 原始数据与中间计算数据 */
	enum feedback_state state;     /**< @brief 当前状态 */
};

/**
 * @brief 绑定编码器操作接口
 * @param[in] feedback 反馈实例
 * @param[in] ops 编码器操作接口
 * @return 无
 */
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops);

/**
 * @brief 绑定反馈参数
 * @param[in] feedback 反馈实例
 * @param[in] param 反馈参数
 * @return 无
 */
void feedback_bind_encoder_param(struct feedback *feedback, struct feedback_param *param);

/**
 * @brief 初始化反馈模块
 * @param[in] feedback 反馈实例
 * @return feedback_error_code 错误码
 * @retval FEEDBACK_ERROR_NONE 初始化成功
 * @retval FEEDBACK_ERROR_PARAM 参数错误
 * @retval FEEDBACK_ERROR_HW_FAILURE 硬件故障
 */
enum feedback_error_code feedback_init(struct feedback *feedback);

/**
 * @brief 更新反馈数据
 * @param[in] feedback 反馈实例
 * @param[in] dt 采样周期，单位：s
 * @return 无
 * @details 执行编码器读取、角度计算、速度计算
 */
void feedback_update(struct feedback *feedback, float dt);

/**
 * @brief 更新反馈原始数据
 * @param[in] feedback 反馈实例
 * @return 无
 */
void feedback_update_raw(struct feedback *feedback);

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
 * @brief 获取线速度
 * @param[in] fb 反馈实例
 * @return float 线速度，单位：m/s
 */
static inline float feedback_get_line_velocity(struct feedback *fb)
{
	return fb->output.velocity_rad_s * fb->param->wheel_radius / fb->param->gear_ratio;
}

/* 以下函数为内部参数更新函数，以 _ 开头 */

/**
 * @brief 更新轮子半径参数
 * @param[in] feedback 反馈实例
 * @param[in] wheel_radius 轮子半径，单位：m
 * @return 无
 */
static inline void _feedback_update_param_wheel_radius(struct feedback *feedback,
						       float wheel_radius)
{
	if (!feedback) {
		return;
	}

	feedback->param->wheel_radius = wheel_radius;
}

/**
 * @brief 更新减速比参数
 * @param[in] feedback 反馈实例
 * @param[in] gear_ratio 减速比
 * @return 无
 */
static inline void _feedback_update_param_gear_ratio(struct feedback *feedback, float gear_ratio)
{
	if (!feedback) {
		return;
	}

	feedback->param->gear_ratio = gear_ratio;
}

/**
 * @brief 更新极对数参数
 * @param[in] feedback 反馈实例
 * @param[in] pole_pairs 极对数
 * @return 无
 */
static inline void _feedback_update_param_pole_pairs(struct feedback *feedback, float pole_pairs)
{
	if (!feedback) {
		return;
	}

	feedback->param->pole_pairs = pole_pairs;
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
 * @param[in] encoder_offset 编码器零位偏移
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

#endif /* FEEDBACK_H */
