#include "_motorlib_internal.h"
#include "foc.h"
#include "motorlib_control_param.h"
#include "close_loop.h"
#include "anticogging.h"
#include <math.h>
#include "motorlib_constants.h"
/* 内部辅助函数声明 */
static bool check_position_stable(struct motor *motor, float target_pos_turns);
static bool check_velocity_stable(struct motor *motor);
static float get_velocity_integrator_output(struct motor *motor);
static void remove_cogging_bias(struct anticogging *anticog);
static int32_t mod_int32(int32_t a, int32_t m);
// static float turns_to_linear(struct motor *motor, float turns);

/**
 * @brief 初始化齿槽校准模块
 * @param[in] motor 电机实例
 */
void anticogging_init(struct motor *motor)
{
	struct anticogging *anticog = &motor->anticoggings;

	anticog->current_index = 0;
	anticog->conut = 0;
	anticog->is_calibrating = false;
	anticog->is_calibrated = false;
	anticog->is_valid = false;

	anticog->tar_pos = 0.0f;
	anticog->stable_counter = 0.0f;
	anticog->position_stable = false;
	anticog->velocity_stable = false;

	/* 初始化补偿表为0 */
	for (int i = 0; i < ANTICOGGING_POINTS_PER_REV; i++) {
		anticog->cogging_map[i] = 0.0f;
	}

	/* 设置默认配置参数 */
	anticog->pos_threshold = ANTICOGGING_POS_THRESHOLD;
	anticog->vel_threshold = ANTICOGGING_VEL_THRESHOLD;
	anticog->stable_time = ANTICOGGING_STABLE_TIME;
}

/**
 * @brief 开始齿槽校准
 * @param[in] motor 电机实例
 */
void anticogging_start_calibration(struct motor *motor)
{
	struct anticogging *anticog = &motor->anticoggings;

	anticog->current_index = 0;
	anticog->conut = 0;
	anticog->is_calibrating = true;
	anticog->is_calibrated = false;
	anticog->is_valid = false;

	anticog->tar_pos = 0.0f;
	anticog->stable_counter = 0.0f;
	anticog->position_stable = false;
	anticog->velocity_stable = false;

	/* 重置积分器，避免历史积分影响 */
	foc_pid_reset(&motor->foc.ctrl.velocity);

	/* 启用位置环控制 */
	/* 注：需要在状态机中设置控制模式 */
}

/**
 * @brief 齿槽校准状态机入口
 * @param[in] sm 状态机实例
 * @note 在20kHz控制循环中周期性调用
 */
void motor_mode_anticogging_calib(struct statemachine *sm)
{
	enum {
		AC_ENTER = USER_STATUS, /* 进入状态 */
		AC_MOVING_TO_POINT,     /* 移动到目标点 */
		AC_WAITING_STABLE,      /* 等待稳定 */
		AC_SAMPLING,            /* 采样保持扭矩 */
		AC_NEXT_POINT,          /* 准备下一个点 */
		AC_FINISHED,            /* 校准完成 */
		AC_EXIT                 /* 退出状态 */
	};

	struct motor *motor = (struct motor *)(sm->data);
	struct foc *foc = &motor->foc;
	struct anticogging *anticog = &motor->anticoggings;

	switch (sm->phase) {
	case ENTER:
		/* 初始化校准 */
		anticogging_start_calibration(motor);
		motor_position_loop_reset(motor);

		/* 设置为位置控制模式（需要根据实际系统实现） */
		/* motor_set_control_mode(motor, CONTROL_MODE_POSITION); */

		sm->phase = AC_MOVING_TO_POINT;
		sm->count = 0;
		break;

	case AC_MOVING_TO_POINT:
		/* 设置目标位置 */
		anticog->tar_pos =
			(float)anticog->current_index / (float)ANTICOGGING_POINTS_PER_REV;

		/* 重置稳定状态 */
		anticog->position_stable = false;
		anticog->velocity_stable = false;
		anticog->stable_counter = 0.0f;
		motor->data.debug.test_value1 = foc->meas.fd_out->mangle_rad;
		/* 进入等待稳定状态 */
		sm->phase = AC_WAITING_STABLE;
		break;

	case AC_WAITING_STABLE:
		/* 执行控制环 */
		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			/* 位置环 */
			float target_linear = motor->data.debug.test_value1;
			foc->ref.velocity =
				motor_position_loop(motor, target_linear, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)SPEED_LOOP_INTERVAL == 0) {
			/* 速度环：使用位置环计算的速度参考值，而不是固定为0 */
			// foc->ref.velocity = motor->data.debug.test_value2;
			motor_velocity_loop(motor, foc->ref.velocity);
		}
		/* 电流环 */
		motor_currment_loop(motor);

		/* 检查稳定条件 */
		// bool pos_stable = check_position_stable(motor, anticog->tar_pos);
		// bool vel_stable = check_velocity_stable(motor);

		// if (pos_stable && vel_stable) {
		// 	anticog->stable_counter += CONTROL_PERIOD_DT;
		// 	if (anticog->stable_counter >= anticog->stable_time) {
		// 		/* 稳定时间达到要求 */
		// 		anticog->position_stable = true;
		// 		anticog->velocity_stable = true;
		// 		sm->phase = AC_SAMPLING;
		// 	}
		// } else {
		// 	/* 不稳定，重置计时器 */
		// 	anticog->stable_counter = 0.0f;
		// }

		sm->count++;
		break;

	case AC_SAMPLING:
		/* 采样速度环积分器输出（保持扭矩） */
		anticog->cogging_map[anticog->current_index] =
			get_velocity_integrator_output(motor);

		/* 采样计数增加 */
		anticog->conut++;

		/* 进入下一个点 */
		sm->phase = AC_NEXT_POINT;
		break;

	case AC_NEXT_POINT:
		/* 移动到下一个校准点 */
		anticog->current_index++;

		if (anticog->current_index >= ANTICOGGING_POINTS_PER_REV) {
			/* 所有点校准完成 */
			sm->phase = AC_FINISHED;
		} else {
			/* 继续下一个点 */
			sm->phase = AC_MOVING_TO_POINT;
		}
		break;

	case AC_FINISHED:
		/* 后处理：零均值化 */
		remove_cogging_bias(anticog);

		/* 标记校准完成 */
		anticog->is_calibrating = false;
		anticog->is_calibrated = true;
		anticog->is_valid = true;

		/* 返回到初始位置 */
		anticog->tar_pos = 0.0f;

		/* 可以在这里保存到Flash */
		/* save_anticogging_to_flash(anticog); */

		sm->phase = AC_EXIT;
		break;

	case AC_EXIT:
		/* 校准完成，可以退出或保持 */
		break;

	default:
		break;
	}
}

/**
 * @brief 获取齿槽补偿值
 * @param[in] motor 电机实例
 * @param[in] pos_estimate 位置估计（转）
 * @return 补偿电流（A）
 * @note 补偿值应加到q轴电流参考值上，用于抵消齿槽效应
 */
float anticogging_get_compensation(struct motor *motor, float pos_estimate)
{
	struct anticogging *anticog = &motor->anticoggings;

	if (!anticog->is_valid) {
		return 0.0f;
	}

	/* 计算位置索引（0~3599） */
	float scaled_pos = pos_estimate * (float)ANTICOGGING_POINTS_PER_REV;
	int32_t index = mod_int32((int32_t)floorf(scaled_pos), ANTICOGGING_POINTS_PER_REV);

	/* 确保索引在有效范围内 */
	if (index < 0 || index >= ANTICOGGING_POINTS_PER_REV) {
		return 0.0f;
	}

	return anticog->cogging_map[index];
}

/**
 * @brief 检查校准是否完成
 * @param[in] motor 电机实例
 * @return true 校准完成，false 进行中或未开始
 */
bool anticogging_is_calibration_done(struct motor *motor)
{
	return motor->anticoggings.is_calibrated;
}

/* ============ 内部辅助函数实现 ============ */

/**
 * @brief 检查位置是否稳定
 * @param target_pos_turns 目标位置（转）
 */
static bool check_position_stable(struct motor *motor, float target_pos_turns)
{
	struct feedback *fb = &motor->feedback;
	struct anticogging *anticog = &motor->anticoggings;

	/* 检查参数有效性 */
	if (!fb->param || fb->param->encoder_resolution == 0) {
		return false;
	}

	/* 获取当前线位移（米）和目标线位移（米） */
	// float current_linear = fb->output.odometer; /* 单位：米 */
	// float target_linear = turns_to_linear(motor, target_pos_turns);
	// float pos_err = target_linear - current_linear;

	/* 计算允许的位置误差（转换为米） */
	/* 位置阈值单位是编码器计数，需要转换为米 */
	// float counts_per_rev = (float)fb->param->encoder_resolution;
	// float meters_per_count = (fb->param->wheel_radius * 0.001f * 2.0f * MOTORLIB_PI) /
	// 			 (counts_per_rev * fb->param->gear_ratio);
	// float pos_threshold_meters = anticog->pos_threshold * meters_per_count;

	// return fabsf(pos_err) <= pos_threshold_meters;
	return 0;
}

/**
 * @brief 检查速度是否稳定
 */
static bool check_velocity_stable(struct motor *motor)
{
	float current_vel = motor->feedback.output.velocity_rad_s;
	return fabsf(current_vel) <= motor->anticoggings.vel_threshold;
}

/**
 * @brief 获取速度环积分器输出
 * @return 速度环积分器输出，单位：A（电流）
 * @note 在稳定状态下，积分器输出代表保持位置所需的电流
 */
static float get_velocity_integrator_output(struct motor *motor)
{
	return motor->foc.ctrl.velocity.integral;
}

/**
 * @brief 移除齿槽表偏置（零均值化）
 */
static void remove_cogging_bias(struct anticogging *anticog)
{
	float sum = 0.0f;

	for (int i = 0; i < ANTICOGGING_POINTS_PER_REV; i++) {
		sum += anticog->cogging_map[i];
	}

	float average = sum / (float)ANTICOGGING_POINTS_PER_REV;

	for (int i = 0; i < ANTICOGGING_POINTS_PER_REV; i++) {
		anticog->cogging_map[i] -= average;
	}
}

/**
 * @brief 整数取模运算，确保结果在 [0, m-1] 范围内
 */
static int32_t mod_int32(int32_t a, int32_t m)
{
	if (m == 0) {
		return 0;
	}

	int32_t r = a % m;
	if (r < 0) {
		r += m;
	}
	return r;
}

// /**
//  * @brief 将旋转位置（转）转换为线位移（米）
//  */
// static float turns_to_linear(struct motor *motor, float turns)
// {
// 	struct feedback_param *param = motor->feedback.param;
// 	if (!param) {
// 		return 0.0f;
// 	}

// 	/* wheel_radius单位：mm */
// 	float radius_m = param->wheel_radius;
// 	float linear = turns * 2.0f * MOTORLIB_PI * radius_m / param->gear_ratio;
// 	return linear;
// }
