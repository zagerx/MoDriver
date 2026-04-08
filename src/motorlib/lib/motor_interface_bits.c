
/**
 * @file motor_interface_bits.c
 * @brief 电机位标志操作实现
 * @details 实现电机状态标志位、错误码、命令位的操作函数
 */

#include "_motorlib_internal.h"
#include "motor_interface_bits.h"

/* ==================== 内部位操作宏 ==================== */

/** @brief 设置状态位 */
#define MOTOR_FLAGS_SET(flags, bit) ((flags) |= (bit))

/** @brief 清除状态位 */
#define MOTOR_FLAGS_CLEAR(flags, bit) ((flags) &= ~(bit))

/** @brief 测试状态位是否置位 */
#define MOTOR_FLAGS_TEST(flags, bit) ((flags) & (bit))

/** @brief 设置错误位 */
#define MOTOR_ERR_SET(err, bit) ((err) |= (bit))

/** @brief 清除错误位 */
#define MOTOR_ERR_CLEAR(err, bit) ((err) &= ~(bit))

/** @brief 测试错误位是否置位 */
#define MOTOR_ERR_TEST(err, bit) ((err) & (bit))

/** @brief 设置命令位 */
#define MOTOR_CMD_SET(cmd, bit) ((cmd) |= (bit))

/** @brief 清除命令位 */
#define MOTOR_CMD_CLEAR(cmd, bit) ((cmd) &= ~(bit))

/** @brief 测试命令位是否置位 */
#define MOTOR_CMD_TEST(cmd, bit) ((cmd) & (bit))

/* ==================== 状态标志位 API ==================== */

int motor_set_flag(struct motor *motor, enum motor_flag_bits bit)
{
	if (motor)
	{
		MOTOR_FLAGS_SET(motor->data.flags, bit);
		return 1;
	}
	return 0;
}

uint32_t motor_get_flag(const struct motor *motor)
{
	return motor ? motor->data.flags : 0;
}

int motor_is_flag_set(const struct motor *motor, enum motor_flag_bits bit)
{
	return motor ? (MOTOR_FLAGS_TEST(motor->data.flags, bit) != 0) : 0;
}

int motor_clear_flag(struct motor *motor, enum motor_flag_bits bit)
{
	if (motor)
	{
		MOTOR_FLAGS_CLEAR(motor->data.flags, bit);
		return 1;
	}
	return 0;
}

int motor_clear_all_flags(struct motor *motor)
{
	if (motor)
	{
		motor->data.flags = 0;
		return 1;
	}
	return 0;
}

/* ==================== 错误码 API ==================== */

int motor_set_error(struct motor *motor, enum motor_errorcode_bits bit)
{
	if (motor)
	{
		MOTOR_ERR_SET(motor->data.errorcode, bit);
		return 1;
	}
	return 0;
}

uint32_t motor_get_errorcode(const struct motor *motor)
{
	return motor ? motor->data.errorcode : 0;
}

int motor_is_error_set(const struct motor *motor, enum motor_errorcode_bits bit)
{
	return motor ? (MOTOR_ERR_TEST(motor->data.errorcode, bit) != 0) : 0;
}

int motor_clear_error(struct motor *motor, enum motor_errorcode_bits bit)
{
	if (motor)
	{
		MOTOR_ERR_CLEAR(motor->data.errorcode, bit);
		return 1;
	}
	return 0;
}

int motor_clear_all_errors(struct motor *motor)
{
	if (motor)
	{
		motor->data.errorcode = 0;
		return 1;
	}
	return 0;
}

/* ==================== 命令位 API ==================== */

int motor_set_command(struct motor *motor, enum motor_command_bits bit)
{
	if (motor)
	{
		MOTOR_CMD_SET(motor->data.command, bit);
		return 1;
	}
	return 0;
}

int motor_clear_command(struct motor *motor, enum motor_command_bits bit)
{
	if (motor)
	{
		MOTOR_CMD_CLEAR(motor->data.command, bit);
		return 1;
	}
	return 0;
}

int motor_is_command_set(const struct motor *motor, enum motor_command_bits bit)
{
	return motor ? (MOTOR_CMD_TEST(motor->data.command, bit) != 0) : 0;
}

int motor_clear_all_commands(struct motor *motor)
{
	if (motor)
	{
		motor->data.command = 0;
		return 1;
	}
	return 0;
}
