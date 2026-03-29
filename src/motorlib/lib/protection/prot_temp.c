/**
 * @file prot_temp.c
 * @brief 温度保护模块（无头文件）
 */

#include "motor.h"
#include "motor_protection.h"
#include "inverter.h"
#include "motor_state.h"

/**
 * @brief 温度保护检测函数
 * @param motor 电机实例
 * @param fault_bit 输出故障位
 * @return true: 触发保护, false: 未触发
 */
bool check_temp(struct motor_t *motor, uint32_t *fault_bit)
{
    struct prot_temp_cfg *cfg = &motor->prot_mgr.temp_cfg;
    
    float temp = motor->data.temperature;
    
    // 过温检测
    if (temp > cfg->overtemp) {
        *fault_bit = FAULT_OVERTEMP;
        return true;
    }
    
    // 低温检测
    if (temp < cfg->lowtemp) {
        *fault_bit = FAULT_UNDERTEMP;
        return true;
    }
    
    return false;
}

/**
 * @brief 温度保护进入故障处理
 * @param motor 电机实例
 * @param fault_bit 故障位
 */
void enter_temp_fault(struct motor_t *motor, uint32_t fault_bit)
{
    (void)fault_bit;
    
    // 过温保护：立即关闭逆变器
    inverter_set_3phase_disable(motor->inverter);
    
    // 切换到故障状态
    TRAN_STATE(motor->state_machine, motor_falut_state);
}

/**
 * @brief 温度保护恢复处理
 * @param motor 电机实例
 */
void recover_temp_fault(struct motor_t *motor)
{
    // 温度恢复：回到空闲状态
    TRAN_STATE(motor->state_machine, motor_idle_state);
}
