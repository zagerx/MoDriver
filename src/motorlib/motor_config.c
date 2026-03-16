#include "_motorlib_internal.h"
#include "inverter.h"
#include "feedback.h"
#include "currsmp.h"
#include "statemachine.h"

/** @brief 电机1反馈实例 */
static struct feedback m1_feedback;

/** @brief 电机1逆变器实例 */
// static struct inverter m1_inverter;

/** @brief 电机1电流采样实例 */
static struct currsmp m1_currsmp;
// /** @brief 电机1配置 */
// static struct motor_config m1_config;

// /** @brief 电机1数据 */
// static struct motor_data m1_data;

/** @brief 电机1状态机 */
static struct statemachine m1_sm;

/** @brief 电机1实例 */
static struct motor motor1 = {
	.feedback = &m1_feedback,
	.inverter = &inverter_1,
	.currsmp = &m1_currsmp,
	.sm = &m1_sm,
	// .config = &m1_config,
	// .data = &m1_data,
};

/** @brief 电机1全局指针 */
struct motor *motor_1 = &motor1;
