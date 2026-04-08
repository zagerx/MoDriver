
/**
 * @file foc.h
 * @brief FOC（磁场定向控制）模块头文件
 * @details 定义FOC测量、参考输入、控制器结构体及电流更新接口
 */

#ifndef FOC_H
#define FOC_H

struct feedback;
struct currsmp;
struct currsmp_output;
struct feedback_output;

#include "foc_pid.h"
/**
 * @brief FOC测量数据结构体
 * @details 存储经过Clarke变换后的α、β轴电流和Park变换后的d、q轴电流
 */
struct foc_measurement {

	struct currsmp_output *cs_out;
	struct feedback_output *fd_out;
	/** α轴电流 */
	float i_alpha;

	/** β轴电流 */
	float i_beta;

	/** d轴电流 */
	float i_d;

	/** q轴电流 */
	float i_q;
};

/**
 * @brief FOC参考输入结构体
 * @details 存储电流环和速度环的目标参考值
 */
struct foc_reference {
	/** d轴电流环输入 */
	float i_d;

	/** q轴电流环输入 */
	float i_q;

	/** 速度环输入，单位：rad/s */
	float velocity;
};

/**
 * @brief FOC控制器结构体
 * @details 包含d/q轴电流环、速度环和位置环的PID控制器
 */
struct foc_control {
	/** d轴电流环PID控制器 */
	struct foc_pid d_axis;

	/** q轴电流环PID控制器 */
	struct foc_pid q_axis;

	/** 速度环PID控制器 */
	struct foc_pid velocity;

	/** 位置环PID控制器 */
	struct foc_pid position;
};

/**
 * @brief FOC主结构体
 * @details 整合FOC控制所需的所有数据和控制器，包括反馈、采样、参数、测量值、参考值和控制器
 */
struct foc {

	/** FOC参数指针 */
	struct foc_param *parm;

	/** FOC测量数据 */
	struct foc_measurement meas;

	/** FOC参考输入 */
	struct foc_reference ref;

	/** FOC控制器 */
	struct foc_control ctrl;

	/** 电机开环强拖自增角度 */
	float self_eangle;
};

/**
 * @brief 绑定FOC数据源
 * @param[in] foc FOC实例指针
 * @param[in] feeback 反馈输出数据指针
 * @param[in] currsmp 电流采样输出数据指针
 * @param[in] foc_param FOC参数指针
 * @return 无
 */
void foc_bind(struct foc *foc, struct feedback *feeback, struct currsmp *currsmp,
	      struct foc_param *foc_param);

/**
 * @brief 更新d轴和q轴电流值
 * @param[in,out] foc FOC实例指针
 * @return 无
 * @note 该函数从电流采样数据更新i_d和i_q值
 */
void foc_update_idiq(struct foc *foc);

#endif
