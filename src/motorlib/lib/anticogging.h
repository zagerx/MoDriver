/**
 * @file anticogging.h
 * @brief 电机齿槽效应补偿校准模块头文件
 * @details 实现基于位置环的齿槽扭矩测量与补偿表生成
 * 采用状态机驱动，在20kHz控制循环中执行
 */

#ifndef ANTICOGGING_H
#define ANTICOGGING_H

#include <stdint.h>
#include <stdbool.h>

/* 齿槽校准配置 */
#define ANTICOGGING_POINTS_PER_REV  360   /* 每转采样点数，1°分辨率（步进大，避免齿槽振荡卡死） */
#define ANTICOGGING_POS_THRESHOLD   0.01f /* 位置稳定阈值（rad），约0.57° */
#define ANTICOGGING_VEL_THRESHOLD   1.0f  /* 速度稳定阈值（rad/s），约5.7°/s */
#define ANTICOGGING_STABLE_TIME     0.5f  /* 稳定时间（秒），确保 PID 收敛 */
#define ANTICOGGING_POINT_TIMEOUT_S 5.0f  /* 单点超时（秒），跳不过就跳过 */

struct motor;
struct statemachine;

/**
 * @brief 齿槽校准错误码定义（按位组合）
 */
enum anticogging_error {
	ANTICOGGING_ERROR_NONE = 0U,
	ANTICOGGING_ERROR_PARAM = 1U << 0,          /**< @brief 参数错误（如编码器分辨率为零） */
	ANTICOGGING_ERROR_TIMEOUT = 1U << 1,        /**< @brief 单点稳定超时 */
	ANTICOGGING_ERROR_NOT_CALIBRATED = 1U << 2, /**< @brief 尚未完成校准 */
	ANTICOGGING_ERROR_ABORTED = 1U << 3,        /**< @brief 校准被外部中断 */
};

/**
 * @brief 齿槽校准数据结构
 */
struct anticogging {
	/* 校准状态 */
	uint32_t current_index; /* 当前校准点索引（0~3599） */
	uint32_t count;         /* 通用计数（兼容原有代码） */
	bool is_calibrating;    /* 正在校准标志 */
	bool is_calibrated;     /* 校准完成标志 */
	bool is_valid;          /* 校准数据有效标志 */

	/* 校准控制 */
	float tar_pos;        /* 目标位置（rad），相对于start_mangle_rad的偏移 */
	float stable_counter; /* 稳定计时器（秒） */

	/* 补偿数据 */
	float cogging_map[ANTICOGGING_POINTS_PER_REV]; /* 3600点补偿表（A） */

	/* 配置参数（可运行时调整） */
	float pos_threshold; /* 位置稳定阈值（rad） */
	float vel_threshold; /* 速度稳定阈值（rad/s） */
	float stable_time;   /* 稳定时间要求（秒） */

	/* 校准基准 */
	float start_mangle_rad; /* 校准起始机械角度（rad），用于计算各点目标位置 */

	/* 调试/错误信息 */
	uint32_t errorcode;    /* 错误码组合值，按 anticogging_error 位定义 */
	float point_wait_time; /* 当前点已等待时间（秒），用于超时检测 */
	uint32_t sample_index; /* 最近一次成功采样的索引 */
};

/**
 * @brief 齿槽校准状态机入口
 * @param[in] sm 状态机实例
 * @note 在20kHz控制循环中周期性调用
 */
void motor_mode_anticogging_calib(struct statemachine *sm);

/**
 * @brief 获取齿槽补偿值
 * @param[in] motor 电机实例
 * @param[in] pos_estimate 位置估计（转）
 * @return 补偿电流（A）
 * @note 在控制循环中调用，根据当前位置查表返回补偿电流值
 *       补偿值应加到q轴电流参考值上
 */
float anticogging_get_compensation(struct motor *motor, float pos_estimate);

/**
 * @brief 初始化齿槽校准模块
 * @param[in] motor 电机实例
 */
void anticogging_init(struct motor *motor);

/**
 * @brief 检查校准是否完成
 * @param[in] motor 电机实例
 * @return true 校准完成，false 进行中或未开始
 */
bool anticogging_is_calibration_done(struct motor *motor);

#endif /* ANTICOGGING_H */
