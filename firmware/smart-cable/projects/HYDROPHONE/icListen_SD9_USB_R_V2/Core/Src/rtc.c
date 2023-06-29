/*
 * rtc.c
 *
 *  Created on: 24 июн. 2022 г.
 *      Author: admin
 */

#include "rtc.h"


extern RTC_HandleTypeDef hrtc;
rtc_typedef rtc;

time_t read_time(rtc_typedef* self_object)
{
	struct tm currTime;
	HAL_RTC_GetTime(&hrtc, &self_object->time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &self_object->date, RTC_FORMAT_BIN);
	currTime.tm_hour=self_object->time.Hours;
	currTime.tm_min=self_object->time.Minutes;
	currTime.tm_sec=self_object->time.Seconds;
	currTime.tm_mday=self_object->date.Date;
	currTime.tm_mon=self_object->date.Month-1;
	currTime.tm_year=(2000+self_object->date.Year)-1900;
	self_object->timestamp=mktime(&currTime);
	return self_object->timestamp;
}

time_t set_time(rtc_typedef* self_object)
{
	HAL_RTC_SetTime(&hrtc, &self_object->time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &self_object->date, RTC_FORMAT_BIN);
}


