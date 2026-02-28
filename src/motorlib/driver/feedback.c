#include "feedback.h"
#include <stdint.h>

void feedback_bind_encoder(struct feedback *feedback, const struct encoder_ops *ops)
{
	if (feedback) {
		feedback->ops = ops;
	}
}
static inline uint16_t feedback_get_raw(struct feedback *fb)
{
	return fb->ops ? fb->ops->read() : 0;
}

void feedback_update(struct feedback *feedback)
{
	feedback->raw = feedback_get_raw(feedback);
}
