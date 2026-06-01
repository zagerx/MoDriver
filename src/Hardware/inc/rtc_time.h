/*
 * RTC 时间服务层
 *
 * @brief  为 CANopen TIME 同步提供极简的 RTC 读写接口
 * @note   时间基准与 CANopen 一致：days + ms since 1984-01-01 00:00:00
 */

#ifndef RTC_TIME_H
#define RTC_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取 RTC 当前时间
 * @param[out] days  自 1984-01-01 以来的天数
 * @param[out] ms    当天午夜以来的毫秒数（0 ~ 86,399,999）
 */
void rtc_get(uint32_t *days, uint32_t *ms);

/**
 * @brief 设置 RTC 时间
 * @param[in] days  自 1984-01-01 以来的天数
 * @param[in] ms    当天午夜以来的毫秒数
 * @return 0 成功，-1 失败
 */
int rtc_set(uint32_t days, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* RTC_TIME_H */
