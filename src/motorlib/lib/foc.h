#ifndef FOC_H
#define FOC_H
#include "foc_data.h"
struct motor;
struct foc {
	struct foc_data data; /**< @brief FOC数据指针 */
};
void foc_bind(struct foc *foc, struct feedback_output *fb_out);
void open_loop_force_align(struct motor *motor, float d_axis_voltage, float eangle);
void open_loop_force_drag(struct motor *motor, float dt, float d_axis_voltage, float omega);

#endif
