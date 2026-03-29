/**
 * @file prot_voltage.c
 * @brief 母线电压保护模块
 */

#include "motor.h"
#include "inverter.h"
#include "currsmp.h"
#include "motor_state.h"
/**
 * @brief 电压保护检测函数
 * @param motor 电机实例
 * @param fault_code 输出故障码
 * @return true: 触发保护, false: 未触发
 */
bool check_voltage(struct motor_t *motor, uint16_t *fault_code)
{
    struct prot_voltage_cfg *cfg = &motor->prot_mgr.voltage_cfg;

    if (!motor->currsmp)
        return false;

    float vbus = get_currsmp_vbus(motor->currsmp);

    if (vbus >= cfg->overvoltage)
    {
        *fault_code = MOTOR_FAULTCODE_OVERVOL;
        return true;
    }
    if (vbus <= cfg->undervoltage)
    {
        *fault_code = MOTOR_FAULTCODE_UNDERVOL;
        return true;
    }
    return false;
}

/**
 * @brief 电压保护进入故障处理
 * @param motor 电机实例
 * @param fault_code 故障码
 */
void enter_voltage_fault(struct motor_t *motor, uint16_t fault_code)
{
    motor->data.faultcode = (enum motor_fault_code)fault_code;
    motor->data.statue = MOTOR_STATE_FAULT;

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
    motor->data.faultcode = MOTOR_FAULTCODE_NOERR;
    motor->data.statue = MOTOR_STATE_IDLE;

    // 电压保护恢复：回到空闲状态
    TRAN_STATE(motor->state_machine, motor_idle_state);
}
