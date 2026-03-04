#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief 编码器操作接口
 *
 * 由Hardware层实现，通过main.c注册到motorlib
 */
struct encoder_ops {
	uint16_t (*read)(void);
};

/**
 * @brief 逆变器操作接口
 *
 * 由Hardware层实现，通过main.c注册到motorlib
 */
struct inverter_ops {
	void (*enable)(void);
	void (*disable)(void);
	void (*set_voltage)(float u, float v, float w);
};

/**
 * @brief 电机硬件接口集合
 *
 * 用于一次性注册所有硬件回调
 */
struct motor_hw_ops {
	const struct encoder_ops *encoder;
	const struct inverter_ops *inverter;
};

/**
 * @brief 反馈参数配置
 */
struct feedback_param {
	float wheel_radius;          /**< 轮子半径 m */
	float gear_ratio;            /**< 减速比 */
	float pole_pairs;            /**< 极对数 */
	float direction;             /**< 旋转方向，1或-1 */
	uint16_t encoder_resolution; /**< 编码器分辨率（CPR） */
	uint16_t encoder_offset;     /**< 编码器零位偏移 */
};

/**
 * @brief 电机扩展参数
 */
struct motor_param_ext {
	struct feedback_param *feedback_param; /**< 反馈参数指针 */
	uint16_t crc_16;                       /**< 参数完整性校验 */
};

#endif /* MOTOR_DRIVER_H */
