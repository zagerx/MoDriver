#ifndef FOC_H
#define FOC_H
#include "feedback.h"
struct motor;
#include "foc_pid.h"
#include "motor_driver.h"
struct foc_measurement {
	// struct currsmp_output *currsmp; /**< @brief 电流采样输出数据 */
	// struct feedback_output *feeback; /**< @brief 反馈输出数据 */
	float i_alpha; /**< @brief α轴电流 */
	float i_beta;  /**< @brief β轴电流 */
	float i_d;     /**< @brief d轴电流 */
	float i_q;     /**< @brief q轴电流 */
};
struct foc_reference {
	float i_d;      /**< @brief d轴电流环输入 */
	float i_q;      /**< @brief q轴电流环输入 */
	float velocity; /**< @brief 速度环输入 rad/s */
};
struct foc_control {
	struct foc_pid d_axis;   /**< @brief d轴电流环PID控制器 */
	struct foc_pid q_axis;   /**< @brief q轴电流环PID控制器 */
	struct foc_pid velocity; /**< @brief 速度环PID控制器 */
	struct foc_pid position; /**< @brief 位置环PID控制器 */
};
struct foc {
	struct feedback *feedback;
	struct currsmp *currsmp;
	struct foc_measurement meas; /**< @brief FOC测量数据 */
	struct foc_reference ref;    /**< @brief FOC参考输入 */
	struct foc_control ctrl;     /**< @brief FOC控制器 */
	float self_eangle;           /**< @brief 电机开环强托自增角度 */
};
/**
 * @brief 绑定FOC数据源
 * @param[in] foc FOC实例
 * @param[in] feeback 反馈输出数据
 * @param[in] currsmp_out 电流采样输出数据
 * @return 无
 */
void foc_bind(struct foc *foc, struct feedback *feeback, struct currsmp *currsmp,
	      struct foc_pid_param *d_axis_pid_param, struct foc_pid_param *q_axis_pid_param,
	      struct foc_pid_param *vel_pid_param, struct foc_pid_param *pos_pid_param);

void foc_update_idiq(struct foc *foc);

#endif
