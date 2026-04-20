
/**
 * @file encoder_calibration.h
 * @brief 编码器校准模块头文件
 * @details 仅包含函数声明，供 calibration.c 或其他模块使用
 */

#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

struct motor;

/**
 * @brief 编码器校准准备
 * @param[in] motor 电机实例
 */
void enc_calib_prepare(struct motor *motor);

/**
 * @brief 编码器对齐单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_align_step(struct motor *motor);

/**
 * @brief 编码器正向扫描单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_scan_forward_step(struct motor *motor);

/**
 * @brief 检查编码器响应和计算极对数/方向
 * @param[in] motor 电机实例
 * @param[out] out_delta 输出扫描累计值
 * @return true=成功, false=失败
 */
bool enc_check_response(struct motor *motor, int32_t *out_delta);

/**
 * @brief 编码器反向扫描单步执行
 * @param[in] motor 电机实例
 * @return 0=继续, 1=完成
 */
int enc_scan_backward_step(struct motor *motor);

/**
 * @brief 计算编码器零点偏移
 * @param[in] motor 电机实例
 * @param[in] scan_delta 正向扫描累计值（保留参数，当前未使用）
 */
void enc_calc_offset(struct motor *motor, int32_t scan_delta);

/**
 * @brief 应用编码器校准结果
 * @param[in] motor 电机实例
 */
void enc_calib_apply(struct motor *motor);

#endif /* ENCODER_CALIBRATION_H */
