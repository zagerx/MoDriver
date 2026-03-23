/**
 * @file prot_stall.c
 * @brief 堵转保护模块
 */

#include "motor.h"
#include "inverter.h"
#include <math.h> // for fabsf
#include "motor_state.h"
/**
 * @brief 堵转保护检测函数
 * @param motor 电机实例
 * @param fault_code 输出故障码
 * @return true: 触发保护, false: 未触发
 */
bool check_stall(struct motor_t *motor, uint16_t *fault_code)
{
    struct prot_stall_cfg *cfg = &motor->prot_mgr.stall_cfg;

    // TODO: 实现堵转检测逻辑
    // 示例：电流大且速度低持续一段时间
    // float current = motor->data.foc_data.meas.idq[1]; // Q轴电流
    // float vel = motor->data.foc_data.meas.vel;
    //
    // if (fabsf(current) > cfg->current_threshold && fabsf(vel) < cfg->vel_threshold) {
    //     *fault_code = MOTOR_FAULTCODE_STALL;
    //     return true;
    // }

    (void)cfg; // 避免未使用警告
    (void)fault_code;
    return false;
}

/**
 * @brief 堵转保护进入故障处理
 * @param motor 电机实例
 * @param fault_code 故障码
 * @note 堵转保护可以有不同的处理策略，比如记录故障次数、降低电流等
 */
void enter_stall_fault(struct motor_t *motor, uint16_t fault_code)
{
    motor->data.faultcode = (enum motor_fault_code)fault_code;
    motor->data.statue = MOTOR_STATE_FAULT;

    // 堵转保护：立即关闭逆变器（后续可增加降流、报警等策略）
    inverter_set_3phase_disable(motor->inverter);

    // 切换到故障状态
    TRAN_STATE(motor->state_machine, motor_falut_state);

    // TODO: 可扩展：记录堵转次数、触发制动等
}

/**
 * @brief 堵转保护恢复处理
 * @param motor 电机实例
 * @note 堵转保护通常需要手动复位，不会自动恢复
 */
void recover_stall_fault(struct motor_t *motor)
{
    motor->data.faultcode = MOTOR_FAULTCODE_NOERR;
    motor->data.statue = MOTOR_STATE_IDLE;

    // 堵转恢复：回到空闲状态
    TRAN_STATE(motor->state_machine, motor_idle_state);

    // TODO: 可扩展：清除故障记录、复位计数器等
}
