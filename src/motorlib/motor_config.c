
/**
 * @file motor_config.c
 * @brief 电机实例配置与全局指针定义
 * @details 定义电机1的静态实例（反馈、逆变器、电流采样）并导出全局指针motor_1
 */

#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "statemachine.h"

/** @brief 电机1反馈实例 */
static struct feedback m1_feedback;

/** @brief 电机1逆变器实例 */
static struct inverter m1_inverter;

/** @brief 电机1电流采样实例 */
static struct currsmp m1_currsmp;

/** @brief 电机1实例 */
static struct motor motor1 = {
	.feedback = &m1_feedback,
	.inverter = &m1_inverter,
	.currsmp = &m1_currsmp,
};

/** @brief 电机1全局指针 */
struct motor *motor_1 = &motor1;
