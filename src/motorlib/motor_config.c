
/**
 * @file motor_config.c
 * @brief 电机实例配置与全局指针定义
 * @details 定义电机1的静态实例（反馈、逆变器、电流采样）并导出全局指针motor_1
 */

#include "_motorlib_internal.h"
/** @brief 电机1实例 */
struct motor motor1 = {0};

/** @brief 电机1全局指针 */
struct motor *motor_1 = &motor1;
