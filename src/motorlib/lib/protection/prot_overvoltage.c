/**
 * @file prot_overvoltage.c
 * @brief 过压保护模块（无头文件）
 */
#include "_motorlib_internal.h"
#include "motor_protection.h"
#include "inverter.h"
#include "currsmp.h"

/**
 * @brief 过压保护检测函数
 * @param motor 电机实例
 * @param fault_bit 输出故障位
 * @return true: 触发保护, false: 未触发
 */
bool check_overvoltage(struct motor *motor, uint32_t *fault_bit)
{
	struct prot_overvoltage_cfg *cfg = &motor->prot_mgr.overvoltage_cfg;

	if (!motor->currsmp) {
		return false;
	}

	float vbus = motor->foc.meas.cs_out->v_bus; // 使用FOC测量的母线电压作为过压判定依据

	if (vbus >= cfg->threshold) {
		*fault_bit = FAULT_OVERVOLTAGE;
		return true;
	}

	return false;
}

/**
 * @brief 过压保护进入故障处理
 * @param motor 电机实例
 * @param fault_bit 故障位
 */
void enter_overvoltage_fault(struct motor *motor, uint32_t fault_bit)
{
	(void)fault_bit;

	// 过压保护：立即关闭逆变器
	inverter_disable(motor->inverter);

	// 切换到故障状态
	// TRAN_STATE(motor->state_machine, motor_falut_state);
}

/**
 * @brief 过压保护恢复处理
 * @param motor 电机实例
 */
void recover_overvoltage_fault(struct motor *motor)
{
	// 过压恢复：回到空闲状态
	// TRAN_STATE(motor->state_machine, motor_idle_state);
	motor_protection_clear_fault(motor, PROT_TYPE_OVERVOLTAGE);
}
