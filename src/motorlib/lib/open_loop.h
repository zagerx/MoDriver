

/**
 * @file open_loop.h
 * @brief 电机开环控制头文件
 * @details 实现电机开环控制功能，包括强制对齐、强制拖动和调试模式
 */

#ifndef OPEN_LOOP_H
#define OPEN_LOOP_H

struct motor;

/**
 * @brief 开环强制对齐（固定角度）
 * @param[in] motor 电机实例
 * @param[in] d_axis_voltage d轴电压
 * @param[in] eangle 电角度
 * @return 无
 */
void open_loop_force_align(struct motor *motor, float d_axis_voltage, float eangle);

/**
 * @brief 开环强制拖动（角速度控制）
 * @param[in] motor 电机实例
 * @param[in] dt 时间间隔
 * @param[in] d_axis_voltage d轴电压
 * @param[in] omega 角速度
 * @return 无
 */
void open_loop_force_drag(struct motor *motor, float dt, float d_axis_voltage, float omega);

/**
 * @brief 获取开环强制角度
 * @param[in] motor 电机实例
 * @return 当前电角度
 */
float open_loop_get_force_angle(struct motor *motor);

/**
 * @brief 开环编码器模式（使用编码器反馈）
 * @param[in] motor 电机实例
 * @param[in] q_axis_voltage q轴电压
 * @return 无
 */
void open_loop_encoder(struct motor *motor, float q_axis_voltage);

/**
 * @brief d轴电流调试模式
 * @param[in] motor 电机实例
 * @param[in] tar 目标电流值
 * @return 无
 */
void currment_debug(struct motor *motor, float tar);

#endif
