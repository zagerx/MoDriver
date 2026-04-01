#ifndef MOTOR_INTERFACE_DRIVER_H
#define MOTOR_INTERFACE_DRIVER_H

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
	void (*enable)(void);                           /**< @brief 使能逆变器 */
	void (*disable)(void);                          /**< @brief 禁用逆变器 */
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
#endif
