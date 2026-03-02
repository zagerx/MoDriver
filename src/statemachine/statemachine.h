#ifndef _STATEMACHINE_H
#define _STATEMACHINE_H
#include <stdint.h>
enum
{
	ENTER = 0,
	EXIT,
	USER_STATUS
};

struct statemachine;
typedef void (*sm_state_t)(struct statemachine *);
enum sm_event
{
	sm_event_0 = 0,
	sm_event_user,
};

typedef struct event_state_table
{
	enum sm_event event;
	sm_state_t state;
} event_state_table_t;
struct statemachine
{
	volatile int16_t phase;
	volatile uint32_t count;
	void *data;
	sm_state_t current_state;
	sm_state_t previous_state;
	struct statemachine *sub_statemachine;
	event_state_table_t *tabel;
	uint16_t tabel_size;
	void (*public_callback)(struct statemachine *);
	int16_t cur_event; // 只能由public_callback函数更改
};

// #define sm_dispatch(me_) ((sm_state_t)(me_)->current_state)((me_))

#define TRAN_STATE(me, targe)                               \
	do                                                  \
	{                                                   \
		(me)->phase = EXIT;                         \
		(me)->current_state(me);                    \
		(me)->previous_state = (me)->current_state; \
		(me)->current_state = (sm_state_t)(targe);  \
		(me)->phase = ENTER;                        \
		(me)->current_state(me);                    \
	} while (0)

void statemachine_init(struct statemachine *obj, void *data, sm_state_t initial_state, event_state_table_t *table, uint16_t tabel_size);
void sm_dispatch(struct statemachine *sm);

#endif /* _STATEMACHINE_H */