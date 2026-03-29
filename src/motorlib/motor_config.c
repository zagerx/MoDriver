/* SPDX-License-Identifier: GPL-2.0 */

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
