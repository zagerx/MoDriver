/**
 * @file motor_interface_driver.h
 * @brief 电机硬件操作接口定义头文件
 * @details 定义编码器和逆变器的抽象操作接口，由 Hardware 层实现并通过 main.c 注册到 motorlib
 * @note 此文件为 Hardware 层与 motorlib 之间的契约，实现这些接口即可适配不同硬件平台
 */

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
