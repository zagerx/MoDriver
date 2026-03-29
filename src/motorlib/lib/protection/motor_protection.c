/**
 * @file motor_protection.c
 * @brief 电机保护管理器核心模块
 */

#include "motor_protection.h"
#include "motor.h"
#include <string.h>

// ========== 外部保护函数声明 ==========
extern bool check_voltage(struct motor_t *motor, uint32_t *fault_bit);
extern void enter_voltage_fault(struct motor_t *motor, uint32_t fault_bit);
extern void recover_voltage_fault(struct motor_t *motor);

extern bool check_stall(struct motor_t *motor, uint32_t *fault_bit);
extern void enter_stall_fault(struct motor_t *motor, uint32_t fault_bit);
extern void recover_stall_fault(struct motor_t *motor);

extern bool check_temp(struct motor_t *motor, uint32_t *fault_bit);
extern void enter_temp_fault(struct motor_t *motor, uint32_t fault_bit);
extern void recover_temp_fault(struct motor_t *motor);

// ========== 辅助函数 ==========

static bool check_trigger(bool *is_triggered, float *debounce_acc, uint16_t debounce_ms, bool triggered, float dt)
{
    if (triggered)
    {
        if (!*is_triggered)
        {
            *debounce_acc += dt;
            if (*debounce_acc * 1000.0f >= debounce_ms)
            {
                *is_triggered = true;
                *debounce_acc = 0;
                return true;
            }
        }
    }
    else
    {
        *debounce_acc = 0;
    }
    return false;
}

static bool check_recover(bool *is_triggered, float *recover_acc, uint16_t recover_ms, float dt)
{
    if (!*is_triggered || recover_ms == 0)
        return false;

    *recover_acc += dt;
    if (*recover_acc * 1000.0f >= recover_ms)
    {
        *is_triggered = false;
        *recover_acc = 0;
        return true;
    }
    return false;
}

// ========== 接口实现 ==========

void motor_protection_init(struct motor_t *motor)
{
    if (!motor)
        return;

    struct protection_manager *mgr = &motor->prot_mgr;

    // 清空所有数据（包括 descs 数组）
    memset(mgr, 0, sizeof(*mgr));

    // 初始化电压保护描述符
    struct protection_desc *v_desc = &mgr->descs[PROT_TYPE_VOLTAGE];
    v_desc->type = PROT_TYPE_VOLTAGE;
    v_desc->fault_bit = FAULT_OVERVOLTAGE; // 过压和欠压共用此位，具体由检测函数判断
    v_desc->debounce_ms = 5;
    v_desc->recover_ms = 1000;
    v_desc->check_fn = check_voltage;
    v_desc->enter_fn = enter_voltage_fault;
    v_desc->recover_fn = recover_voltage_fault;

    // 初始化堵转保护描述符
    struct protection_desc *s_desc = &mgr->descs[PROT_TYPE_STALL];
    s_desc->type = PROT_TYPE_STALL;
    s_desc->fault_bit = FAULT_STALL;
    s_desc->debounce_ms = 500;
    s_desc->recover_ms = 0;
    s_desc->check_fn = check_stall;
    s_desc->enter_fn = enter_stall_fault;
    s_desc->recover_fn = recover_stall_fault;

    // 初始化温度保护描述符
    struct protection_desc *t_desc = &mgr->descs[PROT_TYPE_TEMP];
    t_desc->type = PROT_TYPE_TEMP;
    t_desc->fault_bit = FAULT_OVERTEMP;
    t_desc->debounce_ms = 5000; // 1秒防抖
    t_desc->recover_ms = 3000;
    t_desc->check_fn = check_temp;
    t_desc->enter_fn = enter_temp_fault;
    t_desc->recover_fn = recover_temp_fault;

    // 初始化配置
    mgr->voltage_cfg.overvoltage = 50.0f;
    mgr->voltage_cfg.undervoltage = 46.0f;

    mgr->stall_cfg.current_threshold = 8.0f; // A
    mgr->stall_cfg.vel_threshold = 30.0f;    // 30mm/s

    mgr->temp_cfg.overtemp = 85.0f; // 默认85°C过温
    mgr->temp_cfg.lowtemp = -20.0f; // 默认-20°C低温
}

void motor_protection_update(struct motor_t *motor, float dt)
{
    if (!motor || dt <= 0)
        return;

    struct protection_manager *mgr = &motor->prot_mgr;

    for (int i = PROT_TYPE_NONE + 1; i < PROT_TYPE_COUNT; i++)
    {
        struct protection_desc *desc = &mgr->descs[i];

        if (!desc->check_fn)
            continue;

        uint32_t fault_bit = desc->fault_bit;

        // 检测是否触发（检测函数可修改 fault_bit 以区分子类型，如过压/欠压）
        bool triggered = desc->check_fn(motor, &fault_bit);

        // 触发处理（使用返回的 fault_bit 记录到管理器的位图）
        if (check_trigger(&desc->is_triggered, &desc->debounce_acc, desc->debounce_ms, triggered, dt))
        {
            mgr->fault_bitmap |= fault_bit; // 位或，不覆盖其他故障
            if (desc->enter_fn)
                desc->enter_fn(motor, fault_bit);
        }

        // 恢复处理
        if (!triggered && check_recover(&desc->is_triggered, &desc->recover_acc, desc->recover_ms, dt))
        {
            mgr->fault_bitmap &= ~fault_bit; // 清除该故障位
            if (desc->recover_fn)
                desc->recover_fn(motor);
        }
    }
}

void motor_protection_clear_fault(struct motor_t *motor, enum protection_type type)
{
    if (!motor || type <= PROT_TYPE_NONE || type >= PROT_TYPE_COUNT)
        return;

    struct protection_manager *mgr = &motor->prot_mgr;
    struct protection_desc *desc = &mgr->descs[type];

    desc->is_triggered = false;
    desc->status = PROT_STATUS_NORMAL;
    desc->debounce_acc = 0;
    desc->recover_acc = 0;
    mgr->fault_bitmap &= ~desc->fault_bit; // 清除对应的故障位
}

bool motor_protection_has_fault(struct motor_t *motor)
{
    return motor ? (motor->prot_mgr.fault_bitmap != 0) : false;
}

uint32_t motor_protection_get_faults(struct motor_t *motor)
{
    return motor ? motor->prot_mgr.fault_bitmap : 0;
}
