#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "motor_driver.h"
#include "motorlib_control_param.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** @brief 反馈错误码枚举 */
enum feedback_error_code {
	FEEDBACK_ERROR_NONE = 0,       /**< @brief 无错误 */
	FEEDBACK_ERROR_HW_FAILURE = 1, /**< @brief 硬件故障 */
	FEEDBACK_ERROR_PARAM = 2,      /**< @brief 参数错误 */
};

#if !defined(MOTOR_COUNT) || (MOTOR_COUNT == 0)
#error "MOTOR_COUNT not defined or invalid"
#elif MOTOR_COUNT == 1
extern struct feedback feedback_1;
#elif MOTOR_COUNT == 2
extern struct feedback feedback_1;
extern struct feedback feedback_2;
#else
#error "MOTOR_COUNT must be 1 or 2"
#endif

struct feedback;
/**
 * @brief 绑定编码器操作接口
 * @param[in] feedback 反馈实例
 * @param[in] ops 编码器操作接口
 * @return 无
 */
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops);

/**
 * @brief 绑定反馈参数
 * @param[in] feedback 反馈实例
 * @param[in] param 反馈参数
 * @return 无
 */
void feedback_bind_encoder_param(struct feedback *feedback, struct feedback_param *param);

/**
 * @brief 初始化反馈模块
 * @param[in] feedback 反馈实例
 * @return feedback_error_code 错误码
 * @retval FEEDBACK_ERROR_NONE 初始化成功
 * @retval FEEDBACK_ERROR_PARAM 参数错误
 * @retval FEEDBACK_ERROR_HW_FAILURE 硬件故障
 */
enum feedback_error_code feedback_init(struct feedback *feedback);

/**
 * @brief 更新反馈数据
 * @param[in] feedback 反馈实例
 * @param[in] dt 采样周期 s
 * @return 无
 * @details 执行编码器读取、角度计算、速度计算
 */
void feedback_update(struct feedback *feedback, float dt);
void feedback_update_raw(struct feedback *feedback);
void _feedback_update_param_encoder_offset(struct feedback *feedback, uint16_t encoder_offset);
void _feedback_update_param_encoder_resolution(struct feedback *feedback,
					       uint16_t encoder_resolution);
void _feedback_update_param_direction(struct feedback *feedback, float direction);
void _feedback_update_param_pole_pairs(struct feedback *feedback, float pole_pairs);
void _feedback_update_param_gear_ratio(struct feedback *feedback, float gear_ratio);
void _feedback_update_param_wheel_radius(struct feedback *feedback, float wheel_radius);
float feedback_get_line_velocity(struct feedback *fb);
float feedback_get_velocity(struct feedback *fb);
float feedback_get_elec_angle(struct feedback *fb);
uint16_t feedback_get_raw(struct feedback *fb);

#endif /* FEEDBACK_H */
