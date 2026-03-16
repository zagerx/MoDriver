#ifndef FOC_H
#define FOC_H
#include "foc_data.h"
#include "feedback.h"
struct motor;
struct foc {
	struct feedback *feedback;
	struct currsmp *currsmp;
	struct foc_data data; /**< @brief FOC数据指针 */
};
/**
 * @brief 绑定FOC数据源
 * @param[in] foc FOC实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_bind(struct foc *foc, struct feedback *feeback, struct currsmp *currsmp,
	      struct foc_pid_param *d_axis_pid_param, struct foc_pid_param *q_axis_pid_param,
	      struct foc_pid_param *vel_pid_param, struct foc_pid_param *pos_pid_param);
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
 * @brief 更新id/iq电流
 * @param[in] foc FOC实例
 * @return 无
 */
void foc_update_idiq(struct foc *foc);
void currment_debug(struct motor *motor, float tar);

#endif
