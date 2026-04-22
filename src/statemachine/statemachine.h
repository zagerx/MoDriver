#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H
#include <stdint.h>
enum {
	ENTER = 0,
	EXIT,
	USER_STATUS
};

struct statemachine;
typedef void (*sm_state_t)(struct statemachine *);

struct statemachine {
	volatile int16_t phase;
	volatile uint32_t count;
	void *data;
	sm_state_t current_state;
	sm_state_t previous_state;
	struct statemachine *sub_statemachine;
};

void statemachine_init(struct statemachine *obj, void *data, sm_state_t initial_state);
void sm_dispatch(struct statemachine *sm);
void sm_transition(struct statemachine *sm, sm_state_t new_state);

#endif /* _STATEMACHINE_H */
