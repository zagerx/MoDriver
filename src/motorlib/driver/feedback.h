#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "motor_driver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 反馈输出数据 */
struct feedback_output {
	float eangle_rad;     // 电角度 rad
	float velocity_rad_s; // 机械角速度 rad/s
	float odometer;       // 里程（累积线位移）m
};

/* 反馈参数配置 */
struct feedback_param {
	float wheel_radius;          // 轮子半径 m
	float gear_ratio;            // 减速比
	float pole_pairs;            // 极对数
	float direction;             // 旋转方向，1或-1
	uint16_t encoder_resolution; // 编码器分辨率（CPR）
	uint16_t encoder_offset;     // 编码器零位偏移
};

/* 反馈原始数据与中间计算数据 */
struct feedback_data {
	uint16_t raw;                 // 原始编码器读数
	uint16_t prev_raw;            // 上一次原始读数（用于差分）
	int32_t total_counts;         // 累积计数（考虑溢出）
	float accumulated_mangle_rad; // 累积机械角度 rad
	float prev_mangle_rad;        // 上一次机械角度（用于速度差分）
	float mech_omega_rad_s;       // 机械角速度 rad/s
};

/* 反馈状态 */
enum feedback_state {
	FEEDBACK_STATE_OK = 0,           // 正常
	FEEDBACK_STATE_NOT_CALIBRATED = 1, // 未校准
};

/* 反馈错误码 */
enum feedback_error_code {
	FEEDBACK_ERROR_NONE = 0,      // 无错误
	FEEDBACK_ERROR_HW_FAILURE = 1, // 硬件故障
	FEEDBACK_ERROR_PARAM = 2,     // 参数错误
};

/**
 * @brief 编码器反馈结构体
 */
struct feedback {
	const struct encoder_ops *ops;
	struct feedback_param param;
	struct feedback_output output;
	struct feedback_data data;
	enum feedback_state state;
};

/**
 * @brief 绑定编码器操作接口
 * @param feedback 反馈实例
 * @param ops 编码器操作接口
 */
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops);

/**
 * @brief 初始化反馈模块
 * @param feedback 反馈实例
 * @return 错误码
 */
enum feedback_error_code feedback_init(struct feedback *feedback);

/**
 * @brief 更新反馈数据（编码器读取 + 角度/速度计算）
 * @param feedback 反馈实例
 * @param dt 采样周期 s
 */
void feedback_update(struct feedback *feedback, float dt);

/* 获取编码器原始值 */
static inline uint16_t feedback_get_raw(struct feedback *fb)
{
	return fb->ops ? fb->ops->read() : 0;
}

/* 获取电角度 */
static inline float feedback_get_elec_angle(struct feedback *fb)
{
	return fb->output.eangle_rad;
}

/* 获取机械角速度 */
static inline float feedback_get_velocity(struct feedback *fb)
{
	return fb->output.velocity_rad_s;
}

/* 获取线速度 m/s */
static inline float feedback_get_line_velocity(struct feedback *fb)
{
	return fb->output.velocity_rad_s * fb->param.wheel_radius / fb->param.gear_ratio;
}

/* 更新轮子半径参数 */
static inline void _feedback_update_param_wheel_radius(struct feedback *feedback,
						       float wheel_radius)
{
	if (!feedback) {
		return;
	}
	feedback->param.wheel_radius = wheel_radius;
}

/* 更新减速比参数 */
static inline void _feedback_update_param_gear_ratio(struct feedback *feedback, float gear_ratio)
{
	if (!feedback) {
		return;
	}
	feedback->param.gear_ratio = gear_ratio;
}

/* 更新极对数参数 */
static inline void _feedback_update_param_pole_pairs(struct feedback *feedback, float pole_pairs)
{
	if (!feedback) {
		return;
	}
	feedback->param.pole_pairs = pole_pairs;
}

/* 更新旋转方向参数 */
static inline void _feedback_update_param_direction(struct feedback *feedback, float direction)
{
	if (!feedback) {
		return;
	}
	feedback->param.direction = direction;
}

/* 更新编码器分辨率参数 */
static inline void _feedback_update_param_encoder_resolution(struct feedback *feedback,
							     uint16_t encoder_resolution)
{
	if (!feedback) {
		return;
	}
	feedback->param.encoder_resolution = encoder_resolution;
}

/* 更新编码器零位偏移参数 */
static inline void _feedback_update_param_encoder_offset(struct feedback *feedback,
							 uint16_t encoder_offset)
{
	if (!feedback) {
		return;
	}
	feedback->param.encoder_offset = encoder_offset;
}

#endif /* FEEDBACK_H */
