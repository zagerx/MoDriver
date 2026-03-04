#include "foc.h"

#include "foc_data.h"

void foc_bind(struct foc *foc, struct feedback_output *fb_out)
{
	if (foc) {
		foc_data_bind(&foc->data, fb_out);
	}
}
