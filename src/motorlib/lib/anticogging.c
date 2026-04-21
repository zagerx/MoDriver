#include "_motorlib_internal.h"
#include "foc.h"
#include "motorlib_control_param.h"
#include "close_loop.h"
#include "anticogging.h"
#include <math.h>
#include "motorlib_constants.h"
/* 内部辅助函数声明 */
static void anticogging_start_calibration(struct motor *motor);
static bool check_position_stable(struct motor *motor, float target_mangle_rad);
static bool check_velocity_stable(struct motor *motor);
static float get_velocity_integrator_output(struct motor *motor);
static void remove_cogging_bias(struct anticogging *anticog);
static int32_t mod_int32(int32_t a, int32_t m);

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
	anticog->start_mangle_rad = 0.0f;
	anticog->stable_counter = 0.0f;
	anticog->errorcode = ANTICOGGING_ERROR_NONE;
	anticog->point_wait_time = 0.0f;
	anticog->sample_index = 0;

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
static void anticogging_start_calibration(struct motor *motor)
{
	struct anticogging *anticog = &motor->anticoggings;

	anticog->current_index = 0;
	anticog->conut = 0;
	anticog->is_calibrating = true;
	anticog->is_calibrated = false;
	anticog->is_valid = false;

	anticog->tar_pos = 0.0f;
	anticog->start_mangle_rad = motor->foc.meas.fd_out->mangle_rad;
	anticog->stable_counter = 0.0f;
	anticog->errorcode = ANTICOGGING_ERROR_NONE;
	anticog->point_wait_time = 0.0f;
	anticog->sample_index = 0;

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
		AC_INITIAL_HOLD,        /* 初始保持：tar_pos=0 保持2秒 */
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

		/* 记录校准起始位置 */
		anticog->start_mangle_rad = foc->meas.fd_out->mangle_rad;

		/* 参数合法性检查 */
		if (!motor->feedback.param || motor->feedback.param->encoder_resolution == 0) {
			anticog->errorcode |= ANTICOGGING_ERROR_PARAM;
			sm->phase = AC_EXIT;
			break;
		}

		sm->phase = AC_INITIAL_HOLD;
		sm->count = 0;
		break;

	case AC_INITIAL_HOLD: {
		/* 初始保持阶段：tar_pos = 0，目标位置 = start_mangle_rad，保持2秒 */
		anticog->tar_pos = 0.0f;
		float target_mangle_rad = anticog->start_mangle_rad;

		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			foc->ref.velocity =
				motor_position_loop(motor, target_mangle_rad, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)SPEED_LOOP_INTERVAL == 0) {
			motor_velocity_loop(motor, foc->ref.velocity);
		}
		motor_currment_loop(motor);

		anticog->point_wait_time += CONTROL_PERIOD_DT;
		if (anticog->point_wait_time >= 2.0f) {
			anticog->point_wait_time = 0.0f;
			sm->phase = AC_MOVING_TO_POINT;
		}

		sm->count++;
		break;
	}

	case AC_MOVING_TO_POINT:
		/* 设置目标位置（相对于起始位置的偏移，单位：rad） */
		anticog->tar_pos = (float)anticog->current_index *
				   (MOTORLIB_TWOPI / (float)ANTICOGGING_POINTS_PER_REV);

		/* 重置稳定状态 */
		anticog->stable_counter = 0.0f;
		anticog->point_wait_time = 0.0f;

		/* 进入等待稳定状态 */
		sm->phase = AC_WAITING_STABLE;
		break;

	case AC_WAITING_STABLE: {
		/* 计算当前校准点的目标机械角度 */
		float target_mangle_rad = anticog->start_mangle_rad + anticog->tar_pos;

		/* 执行控制环（位置环→速度环→电流环） */
		if (sm->count % (uint16_t)POSITION_LOOP_INTERVAL == 0) {
			foc->ref.velocity =
				motor_position_loop(motor, target_mangle_rad, POSITION_PERIOD_DT);
		}
		if (sm->count % (uint16_t)SPEED_LOOP_INTERVAL == 0) {
			motor_velocity_loop(motor, foc->ref.velocity);
		}
		/* 电流环 */
		motor_currment_loop(motor);

		/* 检查稳定条件 */
		bool pos_stable = check_position_stable(motor, target_mangle_rad);
		bool vel_stable = check_velocity_stable(motor);
		/* 稳定条件：位置误差和速度都在阈值内 */
		if (pos_stable && vel_stable) {
			anticog->stable_counter += CONTROL_PERIOD_DT;
			if (anticog->stable_counter >= anticog->stable_time) {
				/* 稳定时间达到要求，进入采样 */
				sm->phase = AC_SAMPLING;
			}
		} else {
			/* 不稳定，重置稳定计时器 */
			anticog->stable_counter = 0.0f;
		}

		/* 单点超时检测 */
		anticog->point_wait_time += CONTROL_PERIOD_DT;
		if (anticog->point_wait_time >= ANTICOGGING_POINT_TIMEOUT_S) {
			anticog->errorcode |= ANTICOGGING_ERROR_TIMEOUT;
			/* 超时后跳过该点，继续下一个 */
			sm->phase = AC_NEXT_POINT;
		}

		sm->count++;
		break;
	}

	case AC_SAMPLING:
		/* 采样速度环积分器输出（保持扭矩） */
		anticog->cogging_map[anticog->current_index] =
			get_velocity_integrator_output(motor);

		/* 记录最近一次成功采样的索引 */
		anticog->sample_index = anticog->current_index;

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

		/* 清除超时/参数等瞬时错误，保留调试信息 */
		anticog->errorcode &= ~(ANTICOGGING_ERROR_TIMEOUT | ANTICOGGING_ERROR_ABORTED);

		sm->phase = AC_EXIT;
		break;

	case AC_EXIT:
		/* 校准完成，可以退出或保持 */
		if (!anticog->is_calibrated) {
			anticog->errorcode |= ANTICOGGING_ERROR_NOT_CALIBRATED;
		}
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

	/* pos_estimate 是绝对转数，转换为相对于校准起始位置的偏移 */
	/* 校准时 index=0 对应 start_mangle_rad，运行时也必须基于同一基准 */
	float relative_turns = pos_estimate - anticog->start_mangle_rad / MOTORLIB_TWOPI;

	/* 计算位置索引（0~359） */
	float scaled_pos = relative_turns * (float)ANTICOGGING_POINTS_PER_REV;
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
 * @param[in] motor 电机实例
 * @param[in] target_mangle_rad 目标机械角度（rad）
 * @return true 位置稳定，false 未稳定
 * @note ODrive风格：位置误差转换为编码器计数后与阈值比较
 */
static bool check_position_stable(struct motor *motor, float target_mangle_rad)
{
	struct feedback *fb = &motor->feedback;
	struct anticogging *anticog = &motor->anticoggings;

	/* 使用机械角度误差（rad）直接判定 */
	float pos_err_rad = fabsf(fb->output.mangle_rad - target_mangle_rad);

	return pos_err_rad <= anticog->pos_threshold;
}

/**
 * @brief 检查速度是否稳定
 * @param[in] motor 电机实例
 * @return true 速度稳定，false 未稳定
 * @note ODrive风格：使用PLL速度估计（编码器计数/秒）与阈值比较
 */
static bool check_velocity_stable(struct motor *motor)
{
	struct feedback *fb = &motor->feedback;
	struct anticogging *anticog = &motor->anticoggings;

	/* 使用机械角速度（rad/s）判定 */
	float vel_rad_s = fabsf(fb->output.velocity_rad_s);

	return vel_rad_s <= anticog->vel_threshold;
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
