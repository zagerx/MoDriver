/*
 * CANopen 应用层接口 - STM32 平台
 *
 * @file        CO_app_STM32.h
 * @brief       CANopen 应用层封装，提供类似 motorlib 的 ops 风格接口
 */

#ifndef CO_APP_STM32_H
#define CO_APP_STM32_H

#include <stdint.h>
#include <stdbool.h>
#include "CANopen.h"

/*============================================================================
 * 前向声明
 *===========================================================================*/

struct canopen_app;
typedef struct canopen_app canopen_app_t;

/*============================================================================
 * 应用上下文结构体
 *===========================================================================*/

/**
 * @brief CANopen 应用实例
 * @note 包含配置、回调和内部状态
 */
struct canopen_app {
	/*----- 配置参数（初始化后只读） -----*/
	uint8_t node_id;               /**< 节点 ID (1-127) */
	uint16_t heartbeat_ms;         /**< 心跳周期 (ms) */

	/*----- 内部状态（驱动层管理，应用层勿碰） -----*/
	CO_t *co;                      /**< CANopen 协议栈实例 */
	bool initialized;              /**< 初始化标志 */
	CO_ReturnError_t last_err;     /**< 最后一次错误码 */
};

/*============================================================================
 * API 函数
 *===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 CANopen 应用
 * @param[in,out] app     应用实例（需预先分配内存）
 * @param[in]     node_id 节点 ID (1-127)
 * @return 0 成功，非 0 失败
 *
 * @par 示例:
 * @code
 * static canopen_app_t canopen_app;
 * canopen_app_init(&canopen_app, 21);
 * @endcode
 */
int canopen_app_init(canopen_app_t *app, uint8_t node_id);

/**
 * @brief 反初始化 CANopen 应用
 * @param[in,out] app 应用实例
 * @note 停止 CAN 通信并释放资源
 */
void canopen_app_deinit(canopen_app_t *app);

/**
 * @brief 主循环处理函数（非中断上下文调用）
 * @param[in,out] app 应用实例
 * @param[in] dt_ms 时间差（毫秒），从上一次调用到现在的时间间隔
 * @note 应在主循环中周期性调用，处理 NMT、SDO、心跳等协议功能
 */
void canopen_app_process(canopen_app_t *app, uint32_t dt_ms);

/**
 * @brief 中断处理函数（定时器中断中调用）
 * @param[in,out] app 应用实例
 * @param[in] dt_us 时间差（微秒），从上一次调用到现在的时间间隔
 * @note 在定时器中断中调用，处理 SYNC/RPDO/TPDO
 *       必须保证实时性，不要在中断中执行耗时操作
 */
void canopen_app_interrupt(canopen_app_t *app, uint32_t dt_us);

/**
 * @brief 通信复位
 * @param[in,out] app 应用实例
 * @return 0 成功，非 0 失败
 * @note 用于 NMT 复位通信命令后的重新初始化
 */
int canopen_app_reset_comm(canopen_app_t *app);

/**
 * @brief 获取当前 NMT 状态
 * @param[in] app 应用实例
 * @return 当前 NMT 状态
 */
static inline CO_NMT_internalState_t canopen_app_get_nmt_state(const canopen_app_t *app)
{
	if (app && app->co && app->co->NMT) {
		return CO_NMT_getInternalState(app->co->NMT);
	}
	return CO_NMT_INITIALIZING;
}

/**
 * @brief 获取节点 ID
 * @param[in] app 应用实例
 * @return 当前节点 ID
 */
static inline uint8_t canopen_app_get_node_id(const canopen_app_t *app)
{
	return app ? app->node_id : 0;
}

/**
 * @brief 检查是否初始化完成
 * @param[in] app 应用实例
 * @return true 已初始化
 */
static inline bool canopen_app_is_ready(const canopen_app_t *app)
{
	return app ? app->initialized : false;
}



/**
 * @brief 获取 CANopen 协议栈实例
 * @param[in] app 应用实例
 * @return CO_t 指针，可用于直接访问协议栈功能
 * @note 高级用法，谨慎使用
 */
static inline CO_t *canopen_app_get_co_handle(const canopen_app_t *app)
{
	return app ? app->co : NULL;
}

#ifdef __cplusplus
}
#endif

#endif /* CO_APP_STM32_H */
