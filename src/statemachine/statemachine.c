/**
 * @file statemachine.c
 * @brief 轻量级有限状态机框架实现
 */

#include "statemachine.h"
#include <stdint.h>

void statemachine_init(struct statemachine *obj, void *data, sm_state_t initial_state)
{
	if (!obj || !initial_state) {
		return;
	}
	obj->phase = ENTER;
	obj->data = data;
	obj->current_state = initial_state;
	obj->previous_state = NULL;
	obj->sub_statemachine = NULL;
	obj->next_state = NULL;
}

void sm_dispatch(struct statemachine *sm)
{
	if (!sm || !sm->current_state) {
		return;
	}

	/* 若存在 pending transition，原子化完成旧 EXIT → 新 ENTER */
	if (sm->next_state && sm->next_state != sm->current_state) {
		sm_state_t target = sm->next_state;
		sm->next_state = NULL; /* 立即清除，防止回调内再次触发造成递归 */

		sm->phase = EXIT;
		sm->current_state(sm);

		sm->previous_state = sm->current_state;
		sm->current_state = target;

		sm->phase = ENTER;
		sm->current_state(sm);
		return; /* 切换后本次调度结束 */
	}

	sm->current_state(sm);
}

void sm_transition(struct statemachine *sm, sm_state_t new_state)
{
	if (!sm || !new_state || new_state == sm->current_state) {
		return;
	}
	sm->next_state = new_state; /* 单条指令赋值，可在任意中断安全调用 */
}

void sm_transition_sync(struct statemachine *sm, sm_state_t new_state)
{
	if (!sm || !new_state || new_state == sm->current_state) {
		return;
	}
	sm->next_state = new_state;

	/* 立即在本上下文完成切换：EXIT → 更新指针 → ENTER */
	if (sm->current_state) {
		sm->phase = EXIT;
		sm->current_state(sm);
	}
	sm->previous_state = sm->current_state;
	sm->current_state = new_state;
	sm->phase = ENTER;
	sm->current_state(sm);
}
