/**
 * @file prot_overvoltage.c
 * @brief 过压保护模块（无头文件）
 */

#include "motor.h"
#include "motor_protection.h"
#include "inverter.h"
#include "currsmp.h"
#include "motor_state.h"

/**
 * @brief 过压保护检测函数
 * @param motor 电机实例
 * @param fault_bit 输出故障位
 * @return true: 触发保护, false: 未触发
 */
bool check_overvoltage(struct motor_t *motor, uint32_t *fault_bit)
{
    struct prot_overvoltage_cfg *cfg = &motor->prot_mgr.overvoltage_cfg;

    if (!motor->currsmp)
        return false;

    float vbus = get_currsmp_vbus(motor->currsmp);

    if (vbus >= cfg->threshold)
    {
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
void enter_overvoltage_fault(struct motor_t *motor, uint32_t fault_bit)
{
    (void)fault_bit;

    // 过压保护：立即关闭逆变器
    inverter_set_3phase_disable(motor->inverter);
    s_planner_stop(&motor->scp);

    // 切换到故障状态
    // TRAN_STATE(motor->state_machine, motor_falut_state);
}

/**
 * @brief 过压保护恢复处理
 * @param motor 电机实例
 */
void recover_overvoltage_fault(struct motor_t *motor)
{
    // 过压恢复：回到空闲状态
    // TRAN_STATE(motor->state_machine, motor_idle_state);
    motor_protection_clear_fault(motor, PROT_TYPE_OVERVOLTAGE);
}
