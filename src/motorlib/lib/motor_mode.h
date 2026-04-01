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
