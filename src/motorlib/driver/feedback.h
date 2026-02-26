#ifndef FEEDBACK_H
#define FEEDBACK_H
#include <stdint.h>
struct feedback {
	uint16_t (*get_raw)(void);
};

void feedback_register_callback(struct feedback *feedback, uint16_t (*cb)(void));
void feedback_update(struct feedback *feedback);

#endif
