/**
 * @file motor_interface_params.h
 * @brief 电机参数配置结构体定义头文件
 * @details 定义电机运行所需的所有参数结构体，包括反馈参数、电流采样参数、轨迹规划参数及FOC参数
 * @note 这些参数由上层应用配置，通过 motor_bind_param_ext() 绑定到电机实例
 */

#ifndef MOTOR_INTERFACE_PARAMS_H
#define MOTOR_INTERFACE_PARAMS_H

#include <stdint.h>
/**
 * @brief 反馈参数配置结构体
 */
struct feedback_param {
	float wheel_radius;          /**< @brief 轮子半径，单位：mm */
	float gear_ratio;            /**< @brief 减速比 */
	float pole_pairs;            /**< @brief 极对数 */
	float direction;             /**< @brief 旋转方向，1或-1 */
	uint16_t encoder_resolution; /**< @brief 编码器分辨率（CPR） */
	uint16_t encoder_offset;     /**< @brief 编码器零位偏移（整数部分） */
	float encoder_offset_frac;   /**< @brief 编码器零位小数偏移（0~1），用于精确对齐电角度 */
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
};
struct motor_electrical_param {
	float rs;
	float ls;
};
/**
 * @brief 电机扩展参数结构体
 */
struct motor_param_ext {
	struct feedback_param feedback_param;           /**< @brief 反馈参数 */
	struct currsmp_param currsmp_param;             /**< @brief 电流采样参数 */
	struct trajectory_param traj_param;             /**< @brief 轨迹规划参数 */
	struct foc_param foc_param;                     /**< @brief FOC参数 */
	struct motor_electrical_param electrical_param; /**< @brief 电机本体参数 */
	uint16_t crc_16;                                /**< @brief 参数完整性校验 */
};

#endif /* MOTOR_INTERFACE_PARAMS_H */
