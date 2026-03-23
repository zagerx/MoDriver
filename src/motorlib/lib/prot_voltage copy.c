/**
 * @file prot_voltage.c
 * @brief 母线电压保护模块
 */

#include "motor.h"
#include "motor_protection.h"
#include "inverter.h"
#include "currsmp.h"
#include "motor_state.h"

/**
 * @brief 电压保护检测函数
 * @param motor 电机实例
 * @param fault_bit 输出故障位（可修改以区分子类型）
 * @return true: 触发保护, false: 未触发
 */
bool check_voltage(struct motor_t *motor, uint32_t *fault_bit)
{
    struct prot_voltage_cfg *cfg = &motor->prot_mgr.voltage_cfg;

    if (!motor->currsmp)
        return false;

    float vbus = get_currsmp_vbus(motor->currsmp);

    if (vbus >= cfg->overvoltage)
    {
        *fault_bit = FAULT_OVERVOLTAGE;
        return true;
    }
    if (vbus <= cfg->undervoltage)
    {
        *fault_bit = FAULT_UNDERVOLTAGE;
        return true;
    }
    return false;
}

/**
 * @brief 电压保护进入故障处理
 * @param motor 电机实例
 * @param fault_bit 故障位
 */
void enter_voltage_fault(struct motor_t *motor, uint32_t fault_bit)
{
    // 故障位图已在 protection_manager 中记录，此处处理电机状态
    (void)fault_bit;

    // 电压保护：立即关闭逆变器
    inverter_set_3phase_disable(motor->inverter);

    // 切换到故障状态
    TRAN_STATE(motor->state_machine, motor_falut_state);
}

/**
 * @brief 电压保护恢复处理
 * @param motor 电机实例
 */
void recover_voltage_fault(struct motor_t *motor)
{
    // 电压保护恢复：回到空闲状态
    TRAN_STATE(motor->state_machine, motor_idle_state);
}
