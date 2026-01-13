#include "statemachine.h"
#include <cstdint>

void sm_init(struct statemachine *sm, sm_state_t init, void *data)
{
	int i;

	if (!sm || !init) {
		return;
	}

	sm->state = init;
	sm->prev = 0;
	sm->phase = sm_phase_enter;
	sm->data = data;
}

void sm_trans(struct statemachine *sm, sm_state_t next)
{
	if (!sm || !next || sm->state == next) {
		return;
	}

	sm->phase = sm_phase_exit;
	sm->state(sm);

	sm->prev = sm->state;
	sm->state = next;

	sm->phase = sm_phase_enter;
	sm->state(sm);
}

void sm_dispatch(struct statemachine *sm)
{
	int i;

	if (!sm) {
		return;
	}
	if (sm->public_callback) {
		sm->public_callback(sm);
		if (sm->tabel && sm->cur_event > sm_event_0) {
			sm_state_t targe_state = 0;
			for (uint16_t i = 0; i < (sm->tabel_size); i++) {
				if (sm->tabel[i].event == sm->cur_event) {
					targe_state = sm->tabel[i].state;
					break;
				}
			}
			if (targe_state && targe_state != sm->state) {
				sm_trans(sm, targe_state);
				return;
			}
		}
	}
	if (sm->state) {
		sm->state(sm);
	}
}
