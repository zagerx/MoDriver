#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <stdint.h>
#include "motor_driver.h"
/**
 * @brief 编码器反馈结构体
 */
struct feedback {
	const struct encoder_ops *ops;
	uint16_t raw;
};

/**
 * @param feedback 反馈实例
 * @param cb 读取函数
 */
void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops);

/**
 * @brief 更新反馈数据
 *
 * @param feedback 反馈实例
 */
void feedback_update(struct feedback *feedback);

#endif /* FEEDBACK_H */
