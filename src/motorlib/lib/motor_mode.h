/**
 * @file motor_mode.h
 * @brief 电机操作模式状态机头文件
 * @details 定义CiA 402标准操作模式（PP/PV/HM等）及模式切换接口
 */

#ifndef MOTOR_MODE_H
#define MOTOR_MODE_H
struct statemachine;
/**
 * @brief 电机无模式状态处理函数
 * @param[in] sm 状态机实例指针
 * @details 当电机未分配任何操作模式时调用此状态函数
 */
void motor_mode_none(struct statemachine *sm);

#endif
