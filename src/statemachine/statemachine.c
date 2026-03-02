#include "statemachine.h"
#include <stdint.h>

void statemachine_init(struct statemachine *obj, void *data, sm_state_t initial_state,
		       event_state_table_t *table, uint16_t tabel_size)
{
	if (!obj || !initial_state) {
		return;
	}
	obj->phase = ENTER;
	obj->cur_event = sm_event_0;
	obj->data = data;
	obj->current_state = initial_state;
	obj->tabel = table;
	obj->tabel_size = tabel_size;
	return;
}
void sm_dispatch(struct statemachine *sm)
{
	if (!sm) {
		return;
	}
	if (sm->public_callback) {
		sm->public_callback(sm); // 每次只允许产生一个事件
		// 根据事件查找状态迁移表，如果有对应的事件则迁移状态
		if (sm->tabel && sm->cur_event > sm_event_0) {
			sm_state_t targe_state = 0;
			for (uint16_t i = 0; i < (sm->tabel_size); i++) {
				if (sm->tabel[i].event == sm->cur_event) {
					targe_state = sm->tabel[i].state;
					break;
				}
			}
			if (targe_state && targe_state != sm->current_state) {
				TRAN_STATE(sm, targe_state);
				return;
			}
		}
	}
	if (sm->current_state) {
		sm->current_state(sm);
	}
}
