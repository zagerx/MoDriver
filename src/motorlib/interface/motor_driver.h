#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/**
 * @brief 编码器操作接口结构体
 * @details 由Hardware层实现，通过main.c注册到motorlib
 */
struct encoder_ops {
	uint16_t (*read)(void); /**< @brief 读取编码器原始值函数指针 */
};

/**
 * @brief 逆变器操作接口结构体
 * @details 由Hardware层实现，通过main.c注册到motorlib
 */
struct inverter_ops {
	void (*enable)(void);           /**< @brief 使能逆变器 */
	void (*disable)(void);          /**< @brief 禁用逆变器 */
	void (*set_voltage)(float u, float v, float w); /**< @brief 设置三相电压 */
};

/**
 * @brief 电机硬件接口集合结构体
 * @details 用于一次性注册所有硬件回调
 */
struct motor_hw_ops {
	const struct encoder_ops *encoder;   /**< @brief 编码器操作接口 */
	const struct inverter_ops *inverter; /**< @brief 逆变器操作接口 */
};

/**
 * @brief 反馈参数配置结构体
 */
struct feedback_param {
	float wheel_radius;          /**< @brief 轮子半径 m */
	float gear_ratio;            /**< @brief 减速比 */
	float pole_pairs;            /**< @brief 极对数 */
	float direction;             /**< @brief 旋转方向，1或-1 */
	uint16_t encoder_resolution; /**< @brief 编码器分辨率（CPR） */
	uint16_t encoder_offset;     /**< @brief 编码器零位偏移 */
};

/**
 * @brief 电机扩展参数结构体
 */
struct motor_param_ext {
	struct feedback_param *feedback_param; /**< @brief 反馈参数指针 */
	uint16_t crc_16;                       /**< @brief 参数完整性校验 */
};

#endif /* MOTOR_DRIVER_H */
