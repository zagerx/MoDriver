#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H
#pragma once
/**
 * @file motor_interface.h
 * @brief 电机控制接口头文件
 * @details 定义电机信息结构体并包含各接口子模块头文件
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
