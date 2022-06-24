/*
 * rtc.h
 *
 *  Created on: 24 июн. 2022 г.
 *      Author: admin
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_
#include "main.h"
#include "time.h"

typedef struct{
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	time_t timestamp;
} rtc_typedef;


time_t read_timestamp(rtc_typedef* self_object);


#endif /* INC_RTC_H_ */
