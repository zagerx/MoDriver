#include "foc_data.h"

void foc_data_bind(struct foc_data *data, struct feedback_output *fb_out,
		   struct currsmp_output *currsmp_out)
{
	if (data) {
		data->meas.fb_out = fb_out;
		data->meas.currsmp = currsmp_out;
	}
}
