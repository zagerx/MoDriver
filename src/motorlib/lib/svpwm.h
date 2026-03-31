

/**
 * @file svpwm.h
 * @brief 空间矢量脉宽调制（SVPWM）头文件
 * @details 实现三相逆变器的SVPWM控制算法，包括电压限幅、坐标变换和占空比计算
 */

#ifndef SVPWM_H
#define SVPWM_H

/**
 * @brief 电压矢量等比例限幅（第一层保护）
 *
 * @param[in]  vbus 母线电压
 * @param[in,out] ud   d轴电压（输入输出）
 * @param[in,out] uq   q轴电压（输入输出）
 *
 * @return 无
 *
 * @note 保证√(ud²+uq²) ≤ Vbus/√3，相位角不变
 */
void svpwm_limit_voltage(float vbus, float *ud, float *uq);

/**
 * @brief dq轴电压到αβ轴电压的变换和归一化（第二层转换）
 *
 * @param[in]  eangle 电角度（弧度）
 * @param[in]  vbus   母线电压
 * @param[in]  ud     d轴电压
 * @param[in]  uq     q轴电压
 * @param[out] ualpha α轴电压（归一化输出）
 * @param[out] ubeta  β轴电压（归一化输出）
 *
 * @return 无
 *
 * @note 输出模长≤1.0
 */
void svpwm_normalize(float eangle, float vbus, float ud, float uq, float *ualpha, float *ubeta);

/**
 * @brief 七段式SVPWM占空比计算（第三层映射）
 *
 * @param[in]  valpha α轴电压（归一化）
 * @param[in]  vbeta  β轴电压（归一化）
 * @param[out] dabc   三相占空比数组 [da, db, dc]
 *
 * @return 无
 *
 * @note 输出 duty_a/b/c ∈ [0.0, 1.0]
 */
void svpwm_calc_duty(float valpha, float vbeta, float *dabc);

#endif
