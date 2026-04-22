/**
 * @file statemachine.h
 * @brief 轻量级有限状态机框架
 * @details 专为嵌入式硬实时场景设计，支持延迟切换（deferred transition）
 *          以避免中断嵌套导致的状态机重入。
 */

#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H

#include <stdint.h>
#include <stddef.h>

/** @brief 状态机内置阶段 */
enum {
	ENTER = 0,
	EXIT,
	USER_STATUS
};

struct statemachine;
typedef void (*sm_state_t)(struct statemachine *);

/** @brief 状态机实例 */
struct statemachine {
	volatile int16_t phase;                /**< 当前阶段：ENTER / EXIT / 用户自定义 */
	volatile uint32_t count;               /**< 通用计数器，由用户代码自行维护 */
	void *data;                            /**< 用户私有数据 */
	sm_state_t current_state;              /**< 当前状态函数 */
	sm_state_t previous_state;             /**< 上一个状态函数 */
	struct statemachine *sub_statemachine; /**< 子状态机指针（预留，框架不自动调度） */
	sm_state_t next_state;                 /**< 延迟切换目标，NULL 表示无 pending */
};

/** @brief 初始化状态机，所有字段清零，phase 置为 ENTER */
void statemachine_init(struct statemachine *obj, void *data, sm_state_t initial_state);

/**
 * @brief 调度状态机
 * @details 先处理 pending transition（若有），再执行当前状态的 phase 逻辑。
 *          应在固定周期调用（如 FOC 中断）。
 */
void sm_dispatch(struct statemachine *sm);

/**
 * @brief 延迟状态切换（中断安全）
 * @details 仅将目标状态写入 next_state，不执行回调。
 *          真正的 EXIT/ENTER 在下次 sm_dispatch() 时完成。
 *          可在任意中断中调用。
 */
void sm_transition(struct statemachine *sm, sm_state_t new_state);

/**
 * @brief 同步状态切换（立即执行 EXIT/ENTER）
 * @warning 只能在不会被更高优先级中断抢占的上下文调用
 *          （如系统最高优先级中断内或关中断临界区）。
 */
void sm_transition_sync(struct statemachine *sm, sm_state_t new_state);

#endif /* _STATEMACHINE_H */
