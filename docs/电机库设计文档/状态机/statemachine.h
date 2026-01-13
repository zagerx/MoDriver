#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H

#include <cstdint>
#include <stdint.h>

struct statemachine;

typedef void (*sm_state_t)(struct statemachine *);
// typedef void (*public)(void *);
enum sm_phase {
	sm_phase_enter = 0,
	sm_phase_exit = 1,
	sm_phase_run = 0x100,
};

enum sm_prio {
	sm_prio_0 = 0,
	sm_prio_1,
	sm_prio_2,
	sm_prio_max,
};

enum sm_event {
	sm_event_0 = 0,
	sm_event_user,
};

typedef struct event_state_table {
	enum sm_event event;
	sm_state_t state;
} event_state_table_t;

struct statemachine {
	int16_t phase;
	sm_state_t state;
	sm_state_t prev;
	event_state_table_t *tabel;
	uint16_t tabel_size;
	void (*public_callback)(struct statemachine *);
	int16_t cur_event; // 只能由public_callback函数更改
	void *data;
};

void sm_init(struct statemachine *sm, sm_state_t init, void *data);
void sm_dispatch(struct statemachine *sm);
void sm_trans(struct statemachine *sm, sm_state_t next);

#endif
