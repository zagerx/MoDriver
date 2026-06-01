/*
 * RTC 时间服务层实现
 *
 * @brief  封装 HAL RTC，对外暴露 CANopen 兼容的 days + ms 格式
 */

#include "rtc_time.h"
#include "rtc.h"
#include <stdbool.h>

/*============================================================================
 * 内部常量
 *===========================================================================*/

static const uint16_t days_before_month[12] = {
	0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
};

static const uint8_t month_days[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/*============================================================================
 * 内部辅助函数
 *===========================================================================*/

static inline bool is_leap_year(uint16_t year)
{
	return ((year & 3U) == 0U);
}

/**
 * @brief 年月日 → 自 1984-01-01 以来的天数
 */
static uint32_t days_since_1984(uint16_t year, uint8_t month, uint8_t day)
{
	uint32_t y = (uint32_t)(year - 1984U);
	uint32_t m = (uint32_t)(month - 1U);
	uint32_t d = (uint32_t)(day - 1U);

	uint32_t days = y * 365U + (y + 3U) / 4U;
	days += days_before_month[m];
	if (m > 1U && is_leap_year(year)) {
		days += 1U;
	}
	days += d;
	return days;
}

/**
 * @brief 自 1984-01-01 以来的天数 → 年月日
 */
static void days_to_ymd(uint32_t days, uint16_t *year, uint8_t *month, uint8_t *day)
{
	uint16_t y = 1984U;

	while (1) {
		uint32_t ydays = is_leap_year(y) ? 366U : 365U;
		if (days < ydays) {
			break;
		}
		days -= ydays;
		y++;
	}

	uint32_t m = 0;
	while (m < 11U) {
		uint32_t mdays = month_days[m];
		if (m == 1U && is_leap_year(y)) {
			mdays = 29U;
		}
		if (days < mdays) {
			break;
		}
		days -= mdays;
		m++;
	}

	*year = y;
	*month = (uint8_t)(m + 1U);
	*day = (uint8_t)(days + 1U);
}

/**
 * @brief 计算星期几（Zeller 变体，适配 RTC 定义 1=周一...7=周日）
 */
static uint8_t calculate_weekday(uint16_t year, uint8_t month, uint8_t day)
{
	uint32_t days = days_since_1984(year, month, day);
	uint32_t h = (6U + days) % 7U; /* 0=周日, 1=周一...6=周六 */
	if (h == 0U) {
		return 7U;
	}
	return (uint8_t)h;
}

/**
 * @brief SubSeconds → 毫秒
 */
static inline uint16_t subsec_to_ms(uint32_t second_fraction, uint32_t sub_seconds)
{
	if (second_fraction == 0U) {
		return 0U;
	}
	uint32_t diff = second_fraction - sub_seconds;
	return (uint16_t)((diff * 1000U) / (second_fraction + 1U));
}

/*============================================================================
 * 公共接口
 *===========================================================================*/

void rtc_get(uint32_t *days, uint32_t *ms)
{
	if (days == NULL || ms == NULL) {
		return;
	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	/* HAL 约束：先 GetTime，再 GetDate */
	(void)HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	(void)HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	uint16_t year = (uint16_t)(2000U + sDate.Year);
	*days = days_since_1984(year, sDate.Month, sDate.Date);

	uint16_t subsec_ms = subsec_to_ms(sTime.SecondFraction, sTime.SubSeconds);
	*ms = (uint32_t)sTime.Hours * 3600000U +
	      (uint32_t)sTime.Minutes * 60000U +
	      (uint32_t)sTime.Seconds * 1000U +
	      subsec_ms;
}

int rtc_set(uint32_t days, uint32_t ms)
{
	/* ms 超一天直接退出 */
	if (ms >= 86400000U) {
		return -1;
	}

	/* days → 年月日 */
	uint16_t year;
	uint8_t month, day;
	days_to_ymd(days, &year, &month, &day);

	/* 年月日必须在 RTC 硬件范围 2000-2099 内 */
	if (year < 2000U || year > 2099U) {
		return -1;
	}

	/* ms → 时分秒 */
	uint32_t total_seconds = ms / 1000U;
	uint8_t hour = (uint8_t)(total_seconds / 3600U);
	uint8_t minute = (uint8_t)((total_seconds % 3600U) / 60U);
	uint8_t second = (uint8_t)(total_seconds % 60U);

	/* 进入初始化模式，写时间并清零 SubSeconds */
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	HAL_StatusTypeDef status = RTC_EnterInitMode(&hrtc);
	if (status != HAL_OK) {
		__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
		return -1;
	}

	uint32_t tmpreg = (uint32_t)(((uint32_t)RTC_ByteToBcd2(hour)   << RTC_TR_HU_Pos)  |
	                             ((uint32_t)RTC_ByteToBcd2(minute) << RTC_TR_MNU_Pos) |
	                             ((uint32_t)RTC_ByteToBcd2(second) << RTC_TR_SU_Pos));
	WRITE_REG(hrtc.Instance->TR, (tmpreg & RTC_TR_RESERVED_MASK));
	WRITE_REG(hrtc.Instance->SSR, 0xFFFFFFFFU); /* 清零 SubSeconds */

	status = RTC_ExitInitMode(&hrtc);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	if (status != HAL_OK) {
		return -1;
	}

	/* 写日期 */
	RTC_DateTypeDef sDate = {0};
	sDate.Year = (uint8_t)(year - 2000U);
	sDate.Month = month;
	sDate.Date = day;
	sDate.WeekDay = calculate_weekday(year, month, day);

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		return -1;
	}

	return 0;
}
