/**
 * @file motor_protection.c
 * @brief 电机保护管理器核心模块
 */
#include "_motorlib_internal.h"
#include "motor_protection.h"
#include "motorlib_control_param.h"
#include <string.h>

// ========== 外部保护函数声明 ==========
extern bool check_overvoltage(struct motor *motor, uint32_t *fault_bit);
extern void enter_overvoltage_fault(struct motor *motor, uint32_t fault_bit);
extern void recover_overvoltage_fault(struct motor *motor);

extern bool check_undervoltage(struct motor *motor, uint32_t *fault_bit);
extern void enter_undervoltage_fault(struct motor *motor, uint32_t fault_bit);
extern void recover_undervoltage_fault(struct motor *motor);

extern bool check_stall(struct motor *motor, uint32_t *fault_bit);
extern void enter_stall_fault(struct motor *motor, uint32_t fault_bit);
extern void recover_stall_fault(struct motor *motor);

extern bool check_temp(struct motor *motor, uint32_t *fault_bit);
extern void enter_temp_fault(struct motor *motor, uint32_t fault_bit);
extern void recover_temp_fault(struct motor *motor);

// ========== 辅助函数 ==========

/**
 * @brief 辅助函数：检查保护是否触发（防抖处理）
 * @param[in,out] is_triggered 触发状态指针
 * @param[in,out] debounce_acc 防抖时间累积指针
 * @param[in] debounce_ms 防抖时间（毫秒）
 * @param[in] triggered 当前是否满足触发条件
 * @param[in] dt 时间步长（秒）
 * @return true 触发确认，false 未触发
 */
static bool check_trigger(bool *is_triggered, float *debounce_acc, uint16_t debounce_ms,
			  bool triggered, float dt)
{
	if (triggered) {
		if (!*is_triggered) {
			*debounce_acc += dt;
			if (*debounce_acc * 1000.0f >= debounce_ms) {
				*is_triggered = true;
				*debounce_acc = 0;
				return true;
			}
		}
	} else {
		*debounce_acc = 0;
	}
	return false;
}

/**
 * @brief 辅助函数：检查保护是否恢复（恢复时间处理）
 * @param[in,out] is_triggered 触发状态指针
 * @param[in,out] recover_acc 恢复时间累积指针
 * @param[in] recover_ms 恢复时间（毫秒）
 * @param[in] dt 时间步长（秒）
 * @return true 恢复确认，false 未恢复
 */
static bool check_recover(bool *is_triggered, float *recover_acc, uint16_t recover_ms, float dt)
{
	if (!*is_triggered || recover_ms == 0) {
		return false;
	}

	*recover_acc += dt;
	if (*recover_acc * 1000.0f >= recover_ms) {
		*is_triggered = false;
		*recover_acc = 0;
		return true;
	}
	return false;
}

// ========== 接口实现 ==========

/**
 * @brief 初始化电机保护管理器
 * @param[in] motor 电机实例
 * @return 无
 * @details 配置过压、欠压、堵转等保护类型的阈值与回调函数（温度保护当前未启用）
 */
void motor_protection_init(struct motor *motor)
{
	if (!motor) {
		return;
	}

	struct protection_manager *mgr = &motor->prot_mgr;

	// 清空所有数据（包括 descs 数组）
	memset(mgr, 0, sizeof(*mgr));

	// 初始化过压保护描述符
	struct protection_desc *ov_desc = &mgr->descs[PROT_TYPE_OVERVOLTAGE];
	ov_desc->type = PROT_TYPE_OVERVOLTAGE;
	ov_desc->fault_bit = MOTOR_ERROR_OVERVOLTAGE;
	ov_desc->debounce_ms = 200;
	ov_desc->recover_ms = 1000;
	ov_desc->check_fn = check_overvoltage;
	ov_desc->enter_fn = enter_overvoltage_fault;
	ov_desc->recover_fn = recover_overvoltage_fault;

	// 初始化欠压保护描述符
	struct protection_desc *uv_desc = &mgr->descs[PROT_TYPE_UNDERVOLTAGE];
	uv_desc->type = PROT_TYPE_UNDERVOLTAGE;
	uv_desc->fault_bit = MOTOR_ERROR_UNDERVOLTAGE;
	uv_desc->debounce_ms = 200;
	uv_desc->recover_ms = 1000;
	uv_desc->check_fn = check_undervoltage;
	uv_desc->enter_fn = enter_undervoltage_fault;
	uv_desc->recover_fn = recover_undervoltage_fault;

	// 初始化堵转保护描述符
	struct protection_desc *s_desc = &mgr->descs[PROT_TYPE_STALL];
	s_desc->type = PROT_TYPE_STALL;
	s_desc->fault_bit = MOTOR_ERROR_STALL;
	s_desc->debounce_ms = 500;
	s_desc->recover_ms = 5000;
	s_desc->check_fn = check_stall;
	s_desc->enter_fn = enter_stall_fault;
	s_desc->recover_fn = recover_stall_fault;

	// 初始化温度保护描述符
	// struct protection_desc *t_desc = &mgr->descs[PROT_TYPE_TEMP];
	// t_desc->type = PROT_TYPE_TEMP;
	// t_desc->fault_bit = MOTOR_ERROR_OVERTEMP;
	// t_desc->debounce_ms = 3000; // 1秒防抖
	// t_desc->recover_ms = 3000;
	// t_desc->check_fn = check_temp;
	// t_desc->enter_fn = enter_temp_fault;
	// t_desc->recover_fn = recover_temp_fault;

	// 初始化配置
	mgr->overvoltage_cfg.threshold = OVERVOLTAGE_THRESHOLD;   // 过压阈值52V
	mgr->undervoltage_cfg.threshold = UNDERVOLTAGE_THRESHOLD; // 欠压阈值12V

	mgr->stall_cfg.current_threshold = STALL_CURRENT_THRESHOLD; // A
	mgr->stall_cfg.vel_threshold = STALL_VELOCITY_THRESHOLD;    // 30mm/s

	// mgr->temp_cfg.overtemp = 85.0f; // 默认85°C过温
	// mgr->temp_cfg.lowtemp = -20.0f; // 默认-20°C低温
}

/**
 * @brief 更新电机保护状态
 * @param[in] motor 电机实例
 * @param[in] dt 时间步长，单位：s
 * @return 无
 * @details 周期性调用，检测各保护条件并根据防抖/恢复时间进行状态转移
 */
void motor_protection_update(struct motor *motor, float dt)
{
	if (!motor || dt <= 0) {
		return;
	}

	struct protection_manager *mgr = &motor->prot_mgr;

	for (int i = PROT_TYPE_NONE + 1; i < PROT_TYPE_COUNT; i++) {
		struct protection_desc *desc = &mgr->descs[i];

		if (!desc->check_fn) {
			continue;
		}

		uint32_t fault_bit = desc->fault_bit;

		// 检测是否触发（检测函数可修改 fault_bit 以区分子类型，如过压/欠压）
		bool triggered = desc->check_fn(motor, &fault_bit);

		// 触发处理（使用返回的 fault_bit 记录到管理器的位图）
		if (check_trigger(&desc->is_triggered, &desc->debounce_acc, desc->debounce_ms,
				  triggered, dt)) {
			motor->data.errorcode |= fault_bit; // 位或，不覆盖其他故障
			if (desc->enter_fn) {
				desc->enter_fn(motor, fault_bit);
			}
		}

		// 恢复处理
		if (!triggered &&
		    check_recover(&desc->is_triggered, &desc->recover_acc, desc->recover_ms, dt)) {
			motor->data.errorcode &= ~fault_bit; // 清除该故障位
			if (desc->recover_fn) {
				desc->recover_fn(motor);
			}
		}
	}
}

/**
 * @brief 清除指定类型的保护故障
 * @param[in] motor 电机实例
 * @param[in] type 保护类型
 * @return 无
 */
void motor_protection_clear_fault(struct motor *motor, enum protection_type type)
{
	if (!motor || type <= PROT_TYPE_NONE || type >= PROT_TYPE_COUNT) {
		return;
	}

	struct protection_manager *mgr = &motor->prot_mgr;
	struct protection_desc *desc = &mgr->descs[type];

	desc->is_triggered = false;
	desc->status = PROT_STATUS_NORMAL;
	desc->debounce_acc = 0;
	desc->recover_acc = 0;
	motor->data.errorcode &= ~desc->fault_bit; // 清除对应的故障位
}

/**
 * @brief 清除所有保护故障
 * @param[in] motor 电机实例
 * @return 无
 */
void motor_protection_clear_all_faults(struct motor *motor)
{
	if (!motor) {
		return;
	}

	struct protection_manager *mgr = &motor->prot_mgr;

	// 清除所有保护类型的故障状态
	for (int i = PROT_TYPE_NONE + 1; i < PROT_TYPE_COUNT; i++) {
		struct protection_desc *desc = &mgr->descs[i];
		desc->is_triggered = false;
		desc->status = PROT_STATUS_NORMAL;
		desc->debounce_acc = 0;
		desc->recover_acc = 0;
	}

	// 清空错误码
	motor->data.errorcode = 0;
}

/**
 * @brief 检查是否存在活跃故障
 * @param[in] motor 电机实例
 * @return true 存在故障，false 无故障
 */
bool motor_protection_has_fault(struct motor *motor)
{
	return motor ? (motor->data.errorcode != 0) : false;
}

/**
 * @brief 获取当前故障位图
 * @param[in] motor 电机实例
 * @return uint32_t 故障位组合值
 */
uint32_t motor_protection_get_faults(struct motor *motor)
{
	return motor ? motor->data.errorcode : 0;
}
