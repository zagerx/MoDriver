/**
 * @file motor_protection.h
 * @brief 电机保护管理器核心头文件
 */

#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include <stdint.h>
#include <stdbool.h>

// 故障位图定义
#define FAULT_NONE          (0u)
#define FAULT_OVERVOLTAGE   (1u << 0)
#define FAULT_UNDERVOLTAGE  (1u << 1)
#define FAULT_STALL         (1u << 2)
#define FAULT_OVERTEMP      (1u << 3)
#define FAULT_UNDERTEMP     (1u << 4)

// 前置声明
struct motor_t;

/**
 * @brief 保护类型枚举
 */
enum protection_type
{
    PROT_TYPE_NONE = 0,
    PROT_TYPE_VOLTAGE, // 母线电压保护
    PROT_TYPE_STALL,   // 堵转保护
    PROT_TYPE_TEMP,    // 温度保护
    PROT_TYPE_COUNT
};

/**
 * @brief 保护状态
 */
enum protection_status
{
    PROT_STATUS_NORMAL = 0,
    PROT_STATUS_FAULT,
};

/**
 * @brief 保护描述符（每个电机独立实例）
 */
struct protection_desc
{
    enum protection_type type;     // 保护类型
    enum protection_status status; // 当前状态
    float debounce_acc;            // 防抖时间累积（秒）
    float recover_acc;             // 恢复时间累积（秒）
    bool is_triggered;             // 是否触发
    uint32_t fault_bit;            // 故障位（如 FAULT_OVERVOLTAGE）
    uint16_t debounce_ms;          // 防抖时间（毫秒）
    uint16_t recover_ms;           // 恢复时间（毫秒）

    // 函数指针
    bool (*check_fn)(struct motor_t *motor, uint32_t *fault_bit);
    void (*enter_fn)(struct motor_t *motor, uint32_t fault_bit);
    void (*recover_fn)(struct motor_t *motor);
};

/**
 * @brief 电压保护配置
 */
struct prot_voltage_cfg
{
    float overvoltage;  // 过压阈值 (V)
    float undervoltage; // 欠压阈值 (V)
};

/**
 * @brief 堵转保护配置
 */
struct prot_stall_cfg
{
    float current_threshold; // 电流阈值 (A)
    float vel_threshold;     // 速度阈值 (rad/s)
};

/**
 * @brief 温度保护配置
 */
struct prot_temp_cfg
{
    float overtemp;  // 过温阈值 (°C)
    float lowtemp;   // 低温阈值 (°C)
};

/**
 * @brief 保护管理器
 */
struct protection_manager
{
    struct protection_desc descs[PROT_TYPE_COUNT]; // 保护描述符数组（每个电机独立）
    struct prot_voltage_cfg voltage_cfg;           // 电压保护配置
    struct prot_stall_cfg stall_cfg;               // 堵转保护配置
    struct prot_temp_cfg temp_cfg;                 // 温度保护配置
    uint32_t fault_bitmap;                         // 故障位图
};

/* ============ 接口函数 ============ */

/**
 * @brief 初始化电机保护模块
 * @param[in] motor 电机实例指针
 * @details 初始化保护管理器，配置各保护类型的参数和回调函数
 */
void motor_protection_init(struct motor_t *motor);

/**
 * @brief 更新电机保护状态
 * @param[in] motor 电机实例指针
 * @param[in] dt 时间步长
 * @details 周期性调用，检查各保护条件，更新故障状态
 */
void motor_protection_update(struct motor_t *motor, float dt);

/**
 * @brief 清除指定类型的故障
 * @param[in] motor 电机实例指针
 * @param[in] type 保护类型
 * @details 清除指定保护类型的故障标志和状态
 */
void motor_protection_clear_fault(struct motor_t *motor, enum protection_type type);

/**
 * @brief 检查是否有故障
 * @param[in] motor 电机实例指针
 * @return true 有故障，false 无故障
 */
bool motor_protection_has_fault(struct motor_t *motor);

/**
 * @brief 获取故障位图
 * @param[in] motor 电机实例指针
 * @return 故障位图
 * @details 返回当前所有故障状态的位组合
 */
uint32_t motor_protection_get_faults(struct motor_t *motor);

#endif
