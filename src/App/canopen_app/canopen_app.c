/*
 * CANopen 应用层实现 - STM32 平台
 *
 * @file        canopen_app.c
 * @brief       CANopen 应用层封装，实现 ops 风格接口
 */

#include "canopen_app.h"
#include "OD.h"
#include <stdint.h>
#include <string.h>
#include "motor.h"
/*============================================================================
 * 默认配置参数
 *===========================================================================*/

#define DEFAULT_FIRST_HB_TIME        500
#define DEFAULT_SDO_SRV_TIMEOUT_TIME 1000
#define DEFAULT_SDO_CLI_TIMEOUT_TIME 500
#define DEFAULT_SDO_CLI_BLOCK        false
#define NODE_ID                      (21)
/* NMT 控制字 */
#define NMT_CONTROL                  (CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)

/*============================================================================
 * 内部函数声明
 *===========================================================================*/

static int canopen_app_resetCommunication(canopen_app_t *app);

/*============================================================================
 * API 实现
 *===========================================================================*/
#include "CO_storageBlank.h"

/* 存储对象（必须持久存在） */
static CO_storage_t storage;
extern struct motor_param_ext m1_param_ext;

/* 存储条目数组 */
static CO_storage_entry_t storage_entries[] = {
	{
		.addr = &m1_param_ext,                       // 参数地址
		.len = sizeof(m1_param_ext),                 // 参数长度
		.subIndexOD = 3,                             // 对应 OD 0x1010:3（应用参数）
		.attr = CO_storage_cmd | CO_storage_restore, // 支持命令保存和恢复
	},
};

/* 错误码输出 */
static uint32_t storage_error = 0;
/**
 * @brief 初始化 CANopen 应用
 */
int canopen_app_init(canopen_app_t *app, struct motor *motor)
{
	/* 参数检查 */
	if (app == NULL || NODE_ID < 1 || NODE_ID > 127) {
		return -1;
	}

	/* 保存用户设置的回调函数（如果有） */
	void (*saved_sys_reset)(void) = app->sys_reset_ops;

	/* 清零结构体 */
	memset(app, 0, sizeof(canopen_app_t));

	/* 恢复回调函数 */
	app->sys_reset_ops = saved_sys_reset;

	/* 填充配置 */
	app->node_id = NODE_ID;
	app->heartbeat_ms = DEFAULT_FIRST_HB_TIME;

	/* Allocate CANopen object */
	CO_config_t *config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
	static CO_config_t co_config = {0};
	OD_INIT_CONFIG(co_config);
	co_config.CNT_LEDS = 0;
	co_config.CNT_LSS_SLV = 0;
	config_ptr = &co_config;
#endif

	uint32_t heapMemoryUsed;
	app->co = CO_new(config_ptr, &heapMemoryUsed);
	if (app->co == NULL) {
		return -2;
	}

	/* 初始化通信 */
	if (canopen_app_resetCommunication(app) != 0) {
		CO_delete(app->co);
		app->co = NULL;
		return -3;
	}

	app->initialized = true;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	CO_ReturnError_t err = CO_storageBlank_init(&storage,           // 存储对象
						    app->co->CANmodule, // CAN 模块
						    OD_ENTRY_H1010,     // 0x1010 存储参数
						    OD_ENTRY_H1011,     // 0x1011 恢复默认
						    storage_entries,    // 条目数组
						    1,                  // 条目数量
						    &storage_error      // 错误码输出
	);
	(void)err;
#endif

	/* 绑定 CiA 402 实例参数 */
	cia402_params_bind(
		&app->cia402_inst, motor, &OD_RAM.x6040_controlword, &OD_RAM.x6041_statusword,
		&OD_RAM.x6060_modeworld, &OD_RAM.x6061_modeDisplay, &OD_RAM.x60FF_targetVelocity,
		&OD_RAM.x606C_velocity, &OD_RAM.x603F_errorCode, &OD_RAM.x607A_targetPosition,
		&OD_RAM.x6071_targetTorque, &OD_RAM.x6064_position, &OD_RAM.x6077_torque,
		&OD_RAM.x6081_profileVelocity);

	cia402_init(&app->cia402_inst);

	return 0;
}

/**
 * @brief 反初始化 CANopen 应用
 * @note 不停止定时器，定时器由应用层统一管理
 */
void canopen_app_deinit(canopen_app_t *app)
{
	if (app == NULL || !app->initialized) {
		return;
	}

	/* 进入配置模式 */
	CO_CANsetConfigurationMode(NULL);

	if (app->co) {
		CO_CANmodule_disable(app->co->CANmodule);
		CO_delete(app->co);
		app->co = NULL;
	}

	app->initialized = false;
}

/**
 * @brief 主循环处理函数
 */
void canopen_app_process(canopen_app_t *app, uint32_t dt_ms)
{
	if (app == NULL || !app->initialized || app->co == NULL) {
		return;
	}

	if (dt_ms > 0) {
		/* 转换为微秒 */
		uint32_t timeDifference_us = dt_ms * 1000;

		/* 处理 CANopen */
		// CO_NMT_reset_cmd_t reset_status = ;

		/* 处理复位命令 */
		switch (CO_process(app->co, false, timeDifference_us, NULL)) {
		case CO_RESET_COMM:
			/* 通信复位 */
			CO_CANsetConfigurationMode(NULL);
			CO_delete(app->co);
			app->co = NULL;
			app->initialized = false;

			/* 重新初始化 */
			if (canopen_app_init(app, motor_1) != 0) {
				/* 重新初始化失败，标记错误 */
			}
			break;

		case CO_RESET_APP:
			/* 应用复位 - 系统重启 */

			if (app->sys_reset_ops) {
				app->sys_reset_ops();
			}
			break;

		case CO_RESET_NOT: {
			CO_NMT_internalState_t nmt_state = CO_NMT_getInternalState(app->co->NMT);
			if (nmt_state == CO_NMT_OPERATIONAL ||
			    nmt_state == CO_NMT_PRE_OPERATIONAL) {
				cia402_update(&app->cia402_inst, dt_ms);
			}
		} break;
		default:
			break;
		}
	}
}

/**
 * @brief 中断处理函数
 */
void canopen_app_interrupt(canopen_app_t *app, uint32_t dt_us)
{
	if (app == NULL || !app->initialized || app->co == NULL) {
		return;
	}

	/* 检查状态 */
	if (app->co->nodeIdUnconfigured || !app->co->CANmodule->CANnormal) {
		return;
	}

	CO_LOCK_OD(app->co->CANmodule);

	bool_t syncWas = false;
	uint32_t timeDifference_us = dt_us;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
	syncWas = CO_process_SYNC(app->co, timeDifference_us, NULL);
#endif

#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
	CO_process_RPDO(app->co, syncWas, timeDifference_us, NULL);
#endif

#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
	CO_process_TPDO(app->co, syncWas, timeDifference_us, NULL);
#endif

	CO_UNLOCK_OD(app->co->CANmodule);
}

/*============================================================================
 * 内部函数实现
 *===========================================================================*/

/**
 * @brief 复位通信（内部函数）
 */
static int canopen_app_resetCommunication(canopen_app_t *app)
{
	CO_ReturnError_t err;
	uint32_t errInfo = 0;

	/* 停止 CAN */
	if (app->co->CANmodule) {
		app->co->CANmodule->CANnormal = false;
	}

	/* 进入配置模式 */
	CO_CANsetConfigurationMode(NULL);

	if (app->co->CANmodule) {
		CO_CANmodule_disable(app->co->CANmodule);
	}

	/* 初始化 CAN 硬件 */
	err = CO_CANinit(app->co, NULL, 0);
	if (err != CO_ERROR_NO) {
		app->last_err = err;
		return -1;
	}

	/* 初始化 CANopen */
	err = CO_CANopenInit(app->co, NULL,                  /* alternate NMT */
			     NULL,                           /* alternate em */
			     OD,                             /* Object dictionary */
			     NULL,                           /* OD_statusBits */
			     NMT_CONTROL, app->heartbeat_ms, /* firstHBTime_ms */
			     DEFAULT_SDO_SRV_TIMEOUT_TIME,   /* SDOserverTimeoutTime_ms */
			     DEFAULT_SDO_CLI_TIMEOUT_TIME,   /* SDOclientTimeoutTime_ms */
			     DEFAULT_SDO_CLI_BLOCK,          /* SDOclientBlockTransfer */
			     app->node_id, &errInfo);

	if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
		app->last_err = err;
		return -2;
	}

	/* 初始化 PDO */
	err = CO_CANopenInitPDO(app->co, app->co->em, OD, app->node_id, &errInfo);
	if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
		app->last_err = err;
		return -3;
	}

	/* 启动 CAN */
	CO_CANsetNormalMode(app->co->CANmodule);

	return 0;
}
