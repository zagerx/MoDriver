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

#endif /* MOTOR_DRIVER_H */
