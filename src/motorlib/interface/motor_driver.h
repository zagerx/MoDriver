/* SPDX-License-Identifier: GPL-2.0 */

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

/**
 * @brief 反馈参数配置结构体
 */
struct feedback_param {
	float wheel_radius;          /**< @brief 轮子半径，单位：m */
	float gear_ratio;            /**< @brief 减速比 */
	float pole_pairs;            /**< @brief 极对数 */
	float direction;             /**< @brief 旋转方向，1或-1 */
	uint16_t encoder_resolution; /**< @brief 编码器分辨率（CPR） */
	uint16_t encoder_offset;     /**< @brief 编码器零位偏移 */
};

/**
 * @brief 电流采样参数配置结构体
 */
struct currsmp_param {
	uint16_t a_chn_offset; /**< @brief a轴电流采样通道偏移 */
	uint16_t b_chn_offset; /**< @brief b轴电流采样通道偏移 */
	uint16_t c_chn_offset; /**< @brief c轴电流采样通道偏移 */

	float gain_phase; /**< @brief 相电流增益 */
	float gain_i_bus; /**< @brief 母线电流增益 */
	float gain_v_bus; /**< @brief 母线电压增益 */
};

/**
 * @brief 轨迹规划参数结构体
 */
typedef struct trajectory_param {
	float acc_max; /**< @brief 最大加速度 */
	float vmax;    /**< @brief 最大速度 */
} trajectory_param_t;

/**
 * @brief FOC PID参数结构体
 */
struct foc_pid_param {
	float kp;    /**< @brief 比例增益 */
	float ki;    /**< @brief 积分增益 */
	float kd;    /**< @brief 微分增益 */
	float limit; /**< @brief 输出限幅 */
};

/**
 * @brief FOC参数结构体
 */
struct foc_param {
	struct foc_pid_param d_axis; /**< @brief d轴电流环PID参数 */
	struct foc_pid_param q_axis; /**< @brief q轴电流环PID参数 */
	struct foc_pid_param vel;    /**< @brief 速度环PID参数 */
	struct foc_pid_param pos;    /**< @brief 位置环PID参数 */
	float target_pos;            /**< @brief 目标位置，位置模式的输入 */
	float target_vel;            /**< @brief 目标速度，速度模式的输入 */
	float target_torque;         /**< @brief 目标转矩，力矩模式的输入 */
};

/**
 * @brief 电机扩展参数结构体
 */
struct motor_param_ext {
	struct feedback_param feedback_param; /**< @brief 反馈参数 */
	struct currsmp_param currsmp_param;   /**< @brief 电流采样参数 */
	struct trajectory_param traj_param;   /**< @brief 轨迹规划参数 */
	struct foc_param foc_param;           /**< @brief FOC参数 */
	uint16_t crc_16;                      /**< @brief 参数完整性校验 */
};

#endif /* MOTOR_DRIVER_H */
