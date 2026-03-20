/* SPDX-License-Identifier: GPL-2.0 */

#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/**
 * @brief 编码器校准结果结构体
 */
// struct encoder_calib_result {
// 	int pole_pairs;    /**< 检测到的极对数 */
// 	int direction;     /**< 检测到的方向（1或-1） */
// 	uint16_t offset;   /**< 编码器零位偏移（整数部分） */
// 	float offset_frac; /**< 编码器零位偏移（小数部分，0~1） */
// };

/**
 * @brief 编码器校准对象 - 运行时数据
 */
struct encoder_calib {
	uint32_t tick_cnt;       /**< 滴答计数 */
	uint8_t state;           /**< 子状态机状态 */
	uint16_t raw_prev;       /**< 上次编码器原始值（解卷绕用） */
	int32_t raw_delta_acc;   /**< 编码器累计变化量（解卷绕后） */
	uint32_t align_tick_cnt; /**< 对齐阶段计数 */

	int32_t init_enc_val;     /**< 初始编码器值 */
	int64_t encvaluesum;      /**< 编码器值累加和（用于平均） */
	uint32_t num_steps;       /**< 采样步数 */
	float calib_start_eangle; /**< 校准起始电角度 */

	// /* 检测结果 */
	// int detected_pole_pairs;      /**< 检测到的极对数 */
	// int detected_direction;       /**< 检测到的方向 */
	// uint16_t calculated_offset;   /**< 计算得到的零位偏移 */
	// float calculated_offset_frac; /**< 计算得到的零位小数偏移 */
};

/**
 * @brief 编码器校准阶段
 */
enum encoder_calib_state {
	ENC_CALIB_IDLE = 0,          /**< 空闲状态 */
	ENC_CALIB_ALIGN_START,       /**< 首次对齐到起始位置 */
	ENC_CALIB_SCAN_FORWARD,      /**< 正向扫描 */
	ENC_CALIB_CHECK_RESPONSE,    /**< 检查响应和方向 */
	ENC_CALIB_SCAN_BACKWARD,     /**< 反向扫描 */
	ENC_CALIB_CALC_OFFSET,       /**< 计算零点偏移 */
	ENC_CALIB_DONE,              /**< 校准完成 */
	ENC_CALIB_ERROR_NO_RESPONSE, /**< 错误：编码器无响应 */
	ENC_CALIB_ERROR_CPR_MISMATCH /**< 错误：CPR不匹配 */
};

/**
 * @brief 初始化编码器校准
 * @param[in] motor 电机实例
 * @return 无
 * @details 重置校准状态，禁用逆变器，准备开始校准
 */
void encoder_calib_init(struct motor *motor);

/**
 * @brief 执行一次编码器校准步进
 * @param[in] motor 电机实例
 * @return true 校准完成，false 需要继续执行
 * @note 应在高频任务中周期性调用（如20kHz）
 * @details 基于ODrive的校准思想，使用双向扫描和往返平均计算偏移
 */
bool encoder_calib_run(struct motor *motor);

/**
 * @brief 应用编码器校准结果
 * @param[in] motor 电机实例
 * @return 无
 * @note 结果已在校准过程中实时应用，此函数保留用于后续扩展
 */
void encoder_calib_apply(struct motor *motor);

#endif /* ENCODER_CALIBRATION_H */
