#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H
#pragma once
/**
 * @file motor_interface.h
 * @brief 电机控制接口头文件
 * @details 定义电机控制相关的接口函数，供motorlib内部模块调用
 */
#include "motor_interface_driver.h"
#include "motor_interface_params.h"
#include "motor_interface_bits.h"
#include "motor_interface_mode.h"
struct motor_info {
	enum motor_mode mode;
	enum motor_status status;
	uint32_t flags;
	uint32_t errorcode;
	float actual_pos;
	float actual_vel;
	float actual_torque;
};
#endif
