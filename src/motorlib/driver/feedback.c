#include "feedback.h"
#include <stdint.h>
void feedback_register_callback(struct feedback *feedback, uint16_t (*cb)(void))
{
	feedback->get_raw = cb;
}

void feedback_update(struct feedback *feedback)
{
	uint16_t raw = feedback->get_raw();
	//
}
