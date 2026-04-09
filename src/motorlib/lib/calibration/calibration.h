
/**
 * @file calibration.h
 * @brief 电机校准模块头文件
 * @details 实现电机参数自动校准功能，包括电流采样校准、RL校准和编码器校准
 * 架构说明：
 * - 统一状态机在 calibration.c 中管理
 * - 子模块仅提供具体步骤实现，不维护状态
 * - 使用统一错误码便于调试
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/** ========== 统一校准状态（扁平化） ========== */
enum calibration_state {
	CAL_STATE_IDLE = 0, /**< 空闲状态 */
	CAL_STATE_SUCCESS,  /**< 校准成功 */
	CAL_STATE_FAILED,   /**< 校准失败 */

	/* 电流校准子状态 */
	CAL_STATE_CURRENT_INIT,     /**< 电流校准初始化 */
	CAL_STATE_CURRENT_SAMPLING, /**< 电流采样中 */

	/* RL校准子状态 */
	CAL_STATE_RL_INIT,       /**< RL校准初始化 */
	CAL_STATE_RL_RESISTANCE, /**< 电阻测量 */
	CAL_STATE_RL_INDUCTANCE, /**< 电感测量 */

	/* 编码器校准子状态 */
	CAL_STATE_ENC_ALIGN,         /**< 编码器对齐 */
	CAL_STATE_ENC_SCAN_FORWARD,  /**< 正向扫描 */
	CAL_STATE_ENC_CHECK,         /**< 检查响应和方向 */
	CAL_STATE_ENC_SCAN_BACKWARD, /**< 反向扫描 */
	CAL_STATE_ENC_CALC_OFFSET,   /**< 计算零点偏移 */
};

/** ========== 调试用错误码 ========== */
enum calib_error {
	CAL_ERR_NONE = 0,         /**< 无错误 */
	CAL_ERR_RES_RANGE,        /**< 电阻超出有效范围 */
	CAL_ERR_IND_RANGE,        /**< 电感超出有效范围 */
	CAL_ERR_UNBALANCED,       /**< 相不平衡 */
	CAL_ERR_ENC_NO_RESPONSE,  /**< 编码器无响应 */
	CAL_ERR_ENC_CPR_MISMATCH, /**< CPR不匹配 */
};

/** ========== 电流校准数据 ========== */
struct current_calib_data {
	uint32_t sample_cnt;     /**< 采样计数 */
	uint32_t target_samples; /**< 目标采样数 */
	uint32_t sum_a;          /**< a轴累加和 */
	uint32_t sum_b;          /**< b轴累加和 */
	uint32_t sum_c;          /**< c轴累加和 */
};

/** ========== RL校准数据 ========== */
struct rl_calib_data {
	uint32_t sample_cnt;       /**< 采样计数 */
	uint32_t target_samples;   /**< 目标采样数 */
	float current_setpoint;    /**< 目标电流 [A] */
	float voltage_limit;       /**< 最大测试电压 [V] */
	float voltage_accumulator; /**< 电压积分器 [V] */
	float I_beta_accumulator;  /**< 用于相平衡检测 */
	float test_voltage;        /**< 测试电压 [V] */
	float last_I_alpha;        /**< 上次I_alpha */
	float delta_I_sum;         /**< 电流变化累加 */
	bool voltage_polarity;     /**< 电压极性 */
	float measured_resistance; /**< 测量电阻 [Ohm] */
	float measured_inductance; /**< 测量电感 [H] */
};

/** ========== 编码器校准数据 ========== */
struct encoder_calib_data {
	uint32_t tick_cnt;        /**< 滴答计数 */
	uint16_t raw_prev;        /**< 上次编码器原始值 */
	int32_t raw_delta_acc;    /**< 编码器累计变化量 */
	uint32_t align_tick_cnt;  /**< 对齐阶段计数 */
	int32_t init_enc_val;     /**< 初始编码器值 */
	int64_t encvaluesum;      /**< 编码器值累加和 */
	uint32_t num_steps;       /**< 采样步数 */
	float calib_start_eangle; /**< 校准起始电角度 */
	int32_t scan_delta;       /**< 正向扫描累计值 */
};

/** ========== 主校准对象 ========== */
struct calibration {
	enum calibration_state state; /**< 当前校准状态 */
	enum calib_error error_code;  /**< 错误码（调试用） */

	/** 子模块数据 */
	struct current_calib_data curr;
	struct rl_calib_data rl;
	struct encoder_calib_data enc;
};

/**
 * @brief 初始化校准模块
 * @param[in] motor 电机实例
 */
void calibration_init(struct motor *motor);

/**
 * @brief 校准任务主入口
 * @param[in] motor 电机实例
 * @return true=校准完成（成功或失败）, false=进行中
 * @note 应在主循环中周期性调用
 */
bool calibration_task(struct motor *motor);

/**
 * @brief 获取校准错误码（调试用）
 * @param[in] motor 电机实例
 * @return 错误码
 */
enum calib_error calibration_get_error(struct motor *motor);

#endif /* CALIBRATION_H */
