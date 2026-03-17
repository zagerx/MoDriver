/* SPDX-License-Identifier: GPL-2.0 */

#ifndef FOC_TRAJECTORY_PLAN_H
#define FOC_TRAJECTORY_PLAN_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_driver.h"

/**
 * @brief 轨迹执行器状态枚举
 */
typedef enum {
    TRAJ_EXEC_IDLE = 0,     /**< 空闲状态 */
    TRAJ_EXEC_RUNNING,      /**< 运行中 */
    TRAJ_EXEC_STOPPING,     /**< 紧急停止中 */
    TRAJ_EXEC_END,          /**< 执行结束 */
} trajectory_actor_state_t;

/**
 * @brief 轨迹段结构体
 */
typedef struct {
    float t;    /**< 本段持续时间 (s) */
    float a;    /**< 加速度 */
} traj_seg_t;

/**
 * @brief 轨迹执行数据结构体
 */
typedef struct {
    /* ===== 执行状态 ===== */
    int cur_seg;        /**< 当前段索引 */
    int seg_cnt;        /**< 段总数 */
    float seg_time;     /**< 当前段已运行时间 */

    /* ===== 物理状态（连续量）===== */
    float pos;          /**< 当前位置 (m) */
    float vel;          /**< 当前速度 (m/s) */
    float acc;          /**< 当前加速度 (m/s^2) */

    /* ===== 状态机 ===== */
    trajectory_actor_state_t state;     /**< 执行器状态: IDLE / RUNNING / END */

    /* ===== 轨迹段指针 ===== */
    const traj_seg_t *segs;             /**< 轨迹段数组指针 */
} traj_exec_data_t;

/**
 * @brief 轨迹规划状态枚举
 */
typedef enum {
    TRAJ_PLAN_OK = 0,                   /**< 规划成功 */
    TRAJ_PLAN_NOT_NEEDED,               /**< 无需规划 */
    TRAJ_PLAN_ERR_PARAM,                /**< 参数错误 */
    TRAJ_PLAN_ERR_NO_SOLUTION,          /**< 轨迹不可达 */
    TRAJ_PLAN_SEG_TIME_TOO_SHORT,       /**< 段持续时间过短 */
    TRAJ_PLAN_ERR_BUSY,                 /**< 停止过程中不允许操作 */
} traj_plan_status_t;

/**
 * @brief 轨迹规划输入参数结构体
 */
typedef struct {
    float p0;           /**< 起始位置 */
    float v0;           /**< 起始速度 */
    float a0;           /**< 起始加速度 */

    float v;            /**< 当前速度 */
    float acc;          /**< 统一的加减速度（绝对值） */
    float brake_acc;    /**< 紧急刹停加速度 */
} traj_plan_input_t;

/**
 * @brief 轨迹规划输出结构体
 */
typedef struct {
    traj_seg_t segs[7];     /**< 最多7段轨迹 */
    int seg_cnt;            /**< 实际段数 */
} traj_plan_output_t;

/**
 * @brief 轨迹规划数据结构体
 */
typedef struct {
    traj_plan_input_t plan_in;          /**< 规划输入 */
    traj_plan_output_t plan_out;        /**< 规划输出 */
    float pre_pos;                      /**< 上一次的目标位置，仅用于判断是否需要重规划 */
    float act_pos;                      /**< 实际位置 */

    /* ===== 紧急停止专用轨迹段 ===== */
    traj_seg_t emergency_stop_seg;      /**< 用于立即停止的零长度轨迹段 */
} traj_plan_data_t;

/**
 * @brief 轨迹数据结构体
 */
typedef struct {
    traj_plan_data_t plan_data;     /**< 规划数据 */
    traj_exec_data_t exec_data;     /**< 执行数据 */
} trajectory_data_t;

/**
 * @brief 轨迹规划器结构体
 */
struct trajectory_plan {
    trajectory_data_t data;         /**< 轨迹数据 */
    trajectory_param_t *param;      /**< 轨迹参数指针 */
};

/* ================= API ================= */

/**
 * @brief 绑定轨迹参数
 * @param trajectory 轨迹规划器实例指针
 * @param param 轨迹参数指针
 */
void trajectory_planner_bind_param(struct trajectory_plan *trajectory, trajectory_param_t *param);

/**
 * @brief 初始化轨迹规划器
 * @param trajectory 轨迹规划器实例指针
 * @param start_pos 起始位置
 * @param start_vel 起始速度
 * @param start_acc 起始加速度
 * @param exex_cycle 执行周期
 * @return 初始化结果，0表示成功
 */
int trajectory_planner_init(struct trajectory_plan *trajectory, float start_pos, float start_vel,
                            float start_acc, float exex_cycle);

/**
 * @brief 轨迹规划器动作执行
 * @param trajectory 轨迹规划器实例指针
 * @param dt 时间步长
 */
void trajectory_planner_action(struct trajectory_plan *trajectory, const float dt);

/**
 * @brief 更新目标位置和速度
 * @param trajectory 轨迹规划器实例指针
 * @param new_target_pos 新的目标位置
 * @param new_vel 新的目标速度
 * @return 规划状态
 */
traj_plan_status_t trajectory_planner_update_target(struct trajectory_plan *trajectory,
                                                    float new_target_pos, float new_vel);

/**
 * @brief 紧急停止
 * @param trajectory 轨迹规划器实例指针
 */
void trajectory_planner_stop(struct trajectory_plan *trajectory);

/**
 * @brief 读取规划目标位置
 * @param trajectory 轨迹规划器实例指针
 * @return 目标位置
 */
float trajectory_planner_read_plantarget(const struct trajectory_plan *trajectory);

/**
 * @brief 获取当前位置
 * @param trajectory 轨迹规划器实例指针
 * @return 当前位置
 */
float trajectory_planner_get_pos(const struct trajectory_plan *trajectory);

/**
 * @brief 获取当前速度
 * @param trajectory 轨迹规划器实例指针
 * @return 当前速度
 */
float trajectory_planner_get_vel(const struct trajectory_plan *trajectory);

/**
 * @brief 获取当前加速度
 * @param trajectory 轨迹规划器实例指针
 * @return 当前加速度
 */
float trajectory_planner_get_acc(const struct trajectory_plan *trajectory);

#endif /* FOC_TRAJECTORY_PLAN_H */
