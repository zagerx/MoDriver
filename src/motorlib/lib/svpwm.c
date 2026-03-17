// SPDX-License-Identifier: GPL-2.0

// #include "foc_data.h"
#include "arm_math.h"
#include <float.h>

#define SQRT3_OVER_2   0.8660254037844386f // sqrt(3)/2
#define ONE_OVER_SQRT3 0.5773502691896258f // 1/sqrt(3)
#define SQRT3          1.7320508075688772f // sqrt(3)
#define TWO_OVER_3     0.6666666666666666f // 2/3

#undef M_PI
#define M_PI 3.14159265358979323846f

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

/**
 * @brief 安全的数值限制函数 [0.0, 1.0]
 *
 * @param x 输入数值
 * @return float 限制在[0.0, 1.0]范围内的输出值
 *
 * @note 专门处理浮点误差，防止微小溢出
 */
static inline float clamp_0_1(float x)
{
    const float EPS = 1e-7f;

    if (x < 0.0f) {
        if (x > -EPS) {
            return 0.0f;
        }
        return 0.0f;
    }

    if (x > 1.0f) {
        if (x < 1.0f + EPS) {
            return 1.0f;
        }
        return 1.0f;
    }

    return x;
}

/**
 * @brief 安全的平方根计算
 *
 * @param x 输入数值
 * @return float 平方根结果，输入<=0时返回0
 *
 * @note 防止负值因浮点误差导致NaN
 */
static inline float safe_sqrtf(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    return sqrtf(x);
}

/**
 * @brief 电压矢量等比例限幅（第一层保护）
 *
 * @param vbus 母线电压，必须>0
 * @param ud   指向d轴电压的指针，输入为任意物理电压值
 * @param uq   指向q轴电压的指针，输入为任意物理电压值
 *
 * @note 契约：
 *       1. 输入：vbus必须>0，ud/uq为任意物理电压值
 *       2. 处理：计算U_limit = Vbus / sqrt(3)，若模长超出则等比例缩放
 *       3. 输出：保证√(ud²+uq²) ≤ Vbus/√3，相位角不变
 *       4. 异常：vbus≤0时强制输出零
 */
void svpwm_limit_voltage(float vbus, float *ud, float *uq)
{
    // 1. 母线电压异常检查
    if (vbus <= 0.001f) {
        *ud = 0.0f;
        *uq = 0.0f;
        return;
    }

    // 2. 计算线性调制区极限
    float u_limit = vbus * ONE_OVER_SQRT3; // Vbus / sqrt(3)
    float u_limit_sq = u_limit * u_limit;

    // 3. 计算当前矢量模长的平方
    float u_sq = (*ud) * (*ud) + (*uq) * (*uq);

    // 4. 仅当超出极限时才进行缩放
    if (u_sq > u_limit_sq) {
        // 使用安全开方，避免数值误差
        float u_mag = safe_sqrtf(u_sq);

        // 5. 计算缩放系数并等比例缩放
        float factor = u_limit / u_mag;
        *ud *= factor;
        *uq *= factor;
    }

    // 6. 边界：如果输入已经是零，确保输出也是零
    if (fabsf(*ud) < FLT_EPSILON && fabsf(*uq) < FLT_EPSILON) {
        *ud = 0.0f;
        *uq = 0.0f;
    }
}

/**
 * @brief dq轴电压到αβ轴电压的变换和归一化（第二层转换）
 *
 * @param eangle 电角度，单位为弧度
 * @param vbus   母线电压，必须>0
 * @param ud     d轴电压，需已通过svpwm_limit_voltage限幅
 * @param uq     q轴电压，需已通过svpwm_limit_voltage限幅
 * @param ualpha 指向α轴电压输出值的指针，输出模长≤1.0
 * @param ubeta  指向β轴电压输出值的指针，输出模长≤1.0
 *
 * @note 契约：
 *       1. 输入：eangle为弧度，vbus>0，ud/uq已通过svpwm_limit_voltage限幅
 *       2. 处理：
 *          a) 逆park变换：αβ = R(-θ)·dq
 *          b) 归一化：αβ_mod = αβ / (Vbus/√3)
 *       3. 输出：ualpha, ubeta模长≤1.0
 *       4. 异常：vbus过低或为零时输出零矢量
 */
void svpwm_normalize(float eangle, float vbus, float ud, float uq, float *ualpha, float *ubeta)
{
    float sin_val, cos_val;
    float alpha, beta;

    // 1. 逆Park变换（dq -> αβ）
    arm_sin_cos_f32(eangle * RAD_TO_DEG, &sin_val, &cos_val);
    arm_inv_park_f32(ud, uq, &alpha, &beta, sin_val, cos_val);

    // 2. 电压归一化到线性调制区
    if (vbus > 0.001f) {
        // 归一化因子：1.0 / (Vbus / sqrt(3)) = sqrt(3) / Vbus
        float inv_vbase = SQRT3 / vbus;
        alpha *= inv_vbase;
        beta *= inv_vbase;

        // 3. 验证归一化后模长≤1.0（应通过前置限幅保证）
        float v_sq = alpha * alpha + beta * beta;
        if (v_sq > 1.0001f) { // 允许1%的浮点误差容限
            // 如果超出，说明前置限幅有问题，强制限幅
            float v_mag = safe_sqrtf(v_sq);
            float factor = 1.0f / v_mag;
            alpha *= factor;
            beta *= factor;
        }
    } else {
        // 母线电压异常，输出零矢量
        alpha = 0.0f;
        beta = 0.0f;
    }

    // 4. 返回归一化的αβ电压
    *ualpha = alpha;
    *ubeta = beta;

    // 5. 处理零矢量边界条件
    if (fabsf(alpha) < 1e-6f && fabsf(beta) < 1e-6f) {
        *ualpha = 0.0f;
        *ubeta = 0.0f;
    }
}

/**
 * @brief 七段式SVPWM占空比计算（第三层映射）
 *
 * @param valpha α轴电压输入，范围[-1.0, 1.0]
 * @param vbeta  β轴电压输入，范围[-1.0, 1.0]
 * @param dabc   指向占空比输出数组的指针，输出duty_a/b/c ∈ [0.0, 1.0]
 *
 * @note 契约：
 *       1. 输入：v_alpha, v_beta ∈ [-1.0, 1.0]，且√(v_alpha²+v_beta²) ≤ 1.0
 *       2. 处理：基于投影法的七段式SVPWM算法
 *       3. 输出：duty_a/b/c ∈ [0.0, 1.0]，满足对称七段式序列
 *       4. 零矢量：输入(0,0) ⇒ 输出(0.5,0.5,0.5)
 *       5. 扇区边界：60°整数倍处应平滑过渡
 */
void svpwm_calc_duty(float valpha, float vbeta, float *dabc)
{
    // ----------- 1. 零矢量奇点处理 -----------
    const float ZERO_THRESH = 1e-7f;

    if (fabsf(valpha) < ZERO_THRESH && fabsf(vbeta) < ZERO_THRESH) {
        dabc[0] = 0.5f;
        dabc[1] = 0.5f;
        dabc[2] = 0.5f;
        return;
    }

    // ----------- 3. αβ → abc 投影 -----------
    // 假设α轴对齐A相，β轴超前α轴90°
    float va = valpha;                                // A相
    float vb = -0.5f * valpha + SQRT3_OVER_2 * vbeta; // B相
    float vc = -0.5f * valpha - SQRT3_OVER_2 * vbeta; // C相

    // ----------- 4. 寻找三相极值 -----------
    float vmax = va;
    float vmin = va;

    if (vb > vmax) {
        vmax = vb;
    }
    if (vb < vmin) {
        vmin = vb;
    }

    if (vc > vmax) {
        vmax = vc;
    }
    if (vc < vmin) {
        vmin = vc;
    }

    // ----------- 5. 七段式SVPWM核心计算 -----------
    // 计算有效矢量作用时间（归一化到PWM周期）
    // 对于归一化输入：T1+T2 = (vmax - vmin) * (2/3)
    float T1_plus_T2 = (vmax - vmin) * TWO_OVER_3;

    // 钳制在[0,1]内，防止浮点误差
    T1_plus_T2 = clamp_0_1(T1_plus_T2);

    // 零矢量时间
    float T0 = 1.0f - T1_plus_T2;
    T0 = clamp_0_1(T0);

    // 对称零矢量分布
    float offset = 0.5f * T0;

    // ----------- 6. 计算各相占空比 -----------
    // 七段式投影公式：duty_x = (vx - vmin) * (2/3) + T0/2
    float da = (va - vmin) * TWO_OVER_3 + offset;
    float db = (vb - vmin) * TWO_OVER_3 + offset;
    float dc = (vc - vmin) * TWO_OVER_3 + offset;

    dabc[0] = clamp_0_1(da);
    dabc[1] = clamp_0_1(db);
    dabc[2] = clamp_0_1(dc);
}
