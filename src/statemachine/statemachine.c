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

	return;
}
void sm_dispatch(struct statemachine *sm)
{
	if (!sm) {
		return;
	}

	if (sm->current_state) {
		sm->current_state(sm);
	}
}

void sm_transition(struct statemachine *sm, sm_state_t new_state)
{
	if (!sm || !new_state || new_state == sm->current_state) {
		return;
	}
	if (sm->current_state) {
		sm->phase = EXIT;
		sm->current_state(sm);
	}
	sm->previous_state = sm->current_state;
	sm->current_state = new_state;
	sm->phase = ENTER;
	sm->current_state(sm);
}
