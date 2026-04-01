

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
	int32_t *target_pos;         /**< @brief 位置模式目标位置 */
	int32_t *target_vel;         /**< @brief 速度模式目标速度 */
	int32_t *target_torque;      /**< @brief 力矩模式目标转矩 */
};
/**
 * @brief 电机状态标志位定义（按位组合）
 * @note 低16位为动态运行状态，高16位为事件/完成/持久标志
 * @note 配合 MOTOR_STATUS_BIT() 宏使用，status_flag 为 uint32_t 类型
 */
enum motor_status_bits {
	/* ===== 低16位：动态运行状态 ===== */

	/**< @brief 逆变器已使能 */
	MOTOR_STATUS_ENABLED = 0,
	/**< @brief 闭环控制正在运行 */
	MOTOR_STATUS_RUNNING = 1,
	/**< @brief 正在校准中 */
	MOTOR_STATUS_CALIBRATING = 4,
	/**< @brief 轮廓位置模式(PP)运行中 */
	MOTOR_STATUS_MODE_PP_ACTIVE = 5,
	/**< @brief 轮廓速度模式(PV)运行中 */
	MOTOR_STATUS_MODE_PV_ACTIVE = 6,
	/**< @brief 原点回归模式(HM)运行中 */
	MOTOR_STATUS_MODE_HOMING_ACTIVE = 7,
	/**< @brief 开环控制运行中 */
	MOTOR_STATUS_OPEN_LOOP_ACTIVE = 8,
	/**< @brief 目标已到达 */
	MOTOR_STATUS_TARGET_REACHED = 9,
	/**< @brief 电机正在运动 */
	MOTOR_STATUS_MOVING = 10,

	/* bit13~15 预留 */

	/* ===== 高16位：事件/完成/持久标志 ===== */

	/**< @brief 原点回归完成 */
	MOTOR_STATUS_HOMING_DONE = 16,
	/**< @brief 校准完成 */
	MOTOR_STATUS_CALIBRATION_DONE = 17,
	/**< @brief 速度接近零 */
	MOTOR_STATUS_SPEED_ZERO = 18,
	/**< @brief 位置锁存完成 */
	MOTOR_STATUS_POSITION_LATCH_DONE = 22,
	/**< @brief 轨迹规划器运行中 */

	MOTOR_STATUS_TRAJECTORY_BUSY = 23,
	/* bit24~31 预留 */
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
