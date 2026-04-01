/**
 * @file prot_stall.c
 * @brief 堵转保护模块
 */

#include "motor_protection.h"
#include "inverter.h"
#include <math.h> // for fabsf
#include "motor_state.h"
#include "_motorlib_internal.h"
/**
 * @brief 堵转保护检测函数
 * @param motor 电机实例
 * @param fault_bit 输出故障位
 * @return true: 触发保护, false: 未触发
 * @note 堵转判定条件：电流大(Q轴电流超过阈值) 且 速度低(低于阈值)
 */
bool check_stall(struct motor *motor, uint32_t *fault_bit)
{
	struct prot_stall_cfg *cfg = &motor->prot_mgr.stall_cfg;

	// 获取Q轴电流和速度
	float current = motor->foc.meas.i_q; // 使用FOC测量的q轴电流作为堵转判定的电流值
	float vel = motor->foc.meas.fd_out->velocity_rad_s;

	// 堵转判定：电流大且速度低
	if (fabsf(current) > cfg->current_threshold && fabsf(vel) < cfg->vel_threshold) {
		*fault_bit = FAULT_STALL;
		return true;
	}

	return false;
}

/**
 * @brief 堵转保护进入故障处理
 * @param motor 电机实例
 * @param fault_bit 故障位
 * @note 堵转保护可以有不同的处理策略，比如记录故障次数、降低电流等
 */
void enter_stall_fault(struct motor *motor, uint32_t fault_bit)
{
	(void)fault_bit;

	// 堵转保护：立即关闭逆变器（后续可增加降流、报警等策略）
	inverter_disable(motor->inverter);

	// 切换到故障状态
	// TRAN_STATE(motor->state_machine, motor_falut_state);
	// TODO: 可扩展：记录堵转次数、触发制动等
}

/**
 * @brief 堵转保护恢复处理
 * @param motor 电机实例
 * @note 堵转保护通常需要手动复位，不会自动恢复
 */
void recover_stall_fault(struct motor *motor)
{

	// 堵转恢复：回到空闲状态
	// TRAN_STATE(motor->state_machine, motor_idle_state);
	// TODO: 可扩展：清除故障记录、复位计数器等
	//
	motor_protection_clear_fault(motor, PROT_TYPE_STALL);
}
