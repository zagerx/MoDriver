#ifndef FOC_H
#define FOC_H
#include "foc_data.h"
struct foc {
	struct foc_data data; /**< @brief FOC数据指针 */
};
void foc_bind(struct foc *foc, struct feedback_output *fb_out);

#endif
